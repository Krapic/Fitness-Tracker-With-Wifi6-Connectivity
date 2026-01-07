#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <lvgl.h>
#include <lvgl_input_device.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/gpio.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/wifi_credentials.h>
#include <zephyr/net/socket.h>
#include <net/mqtt_helper.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/__assert.h>

#if defined(CONFIG_MQTT_LIB_TLS)
#include <zephyr/net/tls_credentials.h>
#include "ca_certificate.h"
#endif

// #define STEP_THRESHOLD 0.70f
#define STEP_LENGTH_M 0.7f
// #define WEIGHT_KG 70.0f
#define KCAL_PER_STEP 0.04f
#define MIN_STEP_INTERVAL_MS 500
#define STEP_RESET_WINDOW_MS 200
#define MESSAGE_BUFFER_SIZE 128
#define ADC_OFFSET_MV 1650.0f
#define ADC_SCALE_MV 800.0f

// High-pass (moving average) prozor i statistika za dinamički prag
#define SAMPLE_DT_MS 40
#define HP_WIN_SAMPLES 32  // ~1.3s prozor za klizni prosjek
#define STD_WIN_SAMPLES 64 // isti prozor za μ i σ

#define MOTION_STD_MIN 0.04f // minimalna "živost" (σ) da uopće brojimo korake
#define DYN_K 1.5f			 // k u μ + k·σ (1.5–2.5)

// Kadenca i klasifikacija
#define CADENCE_ALPHA 0.25f // EMA faktor (0..1)
#define RUN_SPM_HI 150		// granica (koraka/min) za trčanje
#define RUN_SPM_LO 130		// donja granica (histereza)
#define RUN_SPEED_HI 0.90f	// m/s granica gore (ostavi kao u kodu)
#define RUN_SPEED_LO 0.70f	// m/s granica dolje (histereza)

static float cadence_hz_ema = 0.0f; // EMA kadence (Hz)

// kružni bufferi i sume za HP i σ
static float acc_buf[HP_WIN_SAMPLES];
static int acc_idx = 0;
static float acc_sum = 0.0f;

static float std_buf[STD_WIN_SAMPLES];
static int std_idx = 0;
static float std_sum = 0.0f, std_sq_sum = 0.0f;

// anti-shake i peak prominence
#define PEAK_PROM_MIN 0.12f	  // minimalna "prominencija" peaka u g
#define MIN_VALLEY_GAP_MS 200 // min. razmak od zadnje doline za stabilnu prominenciju

// Warm-up/priming stanja
static bool filters_ready = false;
static uint32_t sample_count = 0;
#define WARMUP_SAMPLES (HP_WIN_SAMPLES > STD_WIN_SAMPLES ? HP_WIN_SAMPLES : STD_WIN_SAMPLES)
#define STARTUP_GRACE_MS 1500

// ADC kanali za X, Y, Z
static const struct adc_dt_spec adc_channel0 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec adc_channel1 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
static const struct adc_dt_spec adc_channel2 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2);

int steps = 0;
float distance_m = 0;
float calories = 0;
float speed_m_s = 0;
const char *activity = "Standing";
bool step_detected = false;
int64_t last_step_time = 0;
int64_t reset_time = 0;
// float previous_total_acc = 0.0f;
bool is_running = false;  // Stanje aktivnosti
float previous_hp = 0.0f; // prethodni high-pass uzorak (za rising-edge)

LOG_MODULE_REGISTER(ADC_over_MQTT, LOG_LEVEL_INF);

#define EVENT_MASK (NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED)

// Komande za kontroliranje LED-ica i tipki
#define LED1_ON_CMD "LED1ON"
#define LED1_OFF_CMD "LED1OFF"
#define LED2_ON_CMD "LED2ON"
#define LED2_OFF_CMD "LED2OFF"
#define BUTTON1_MSG "Button 1 pressed"
#define BUTTON2_MSG "Button 2 pressed"

// Definiranje ID-a poruke koja se koristi kad se pretplati na topic
#define SUBSCRIBE_TOPIC_ID 1234

static struct net_mgmt_event_callback mgmt_cb;
static bool connected;
static K_SEM_DEFINE(run_app, 0, 1);

// Deklaracija varijable za spremanje klijent ID-a
static uint8_t client_id[sizeof(CONFIG_BOARD) + 11];

// Kadenca i klasifikacija
#define CADENCE_ALPHA 0.25f // EMA faktor (0..1)
#define RUN_SPM_HI 150		// granica (koraka/min) za trčanje
#define RUN_SPM_LO 130		// donja granica (histereza)
#define RUN_SPEED_HI 0.90f	// m/s granica gore
#define RUN_SPEED_LO 0.70f	// m/s granica dolje (histereza)

static void net_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
								   struct net_if *iface)
{
	if ((mgmt_event & EVENT_MASK) != mgmt_event)
	{
		return;
	}
	if (mgmt_event == NET_EVENT_L4_CONNECTED)
	{
		LOG_INF("Network connected");
		connected = true;
		k_sem_give(&run_app);
		return;
	}
	if (mgmt_event == NET_EVENT_L4_DISCONNECTED)
	{
		if (connected == false)
		{
			LOG_INF("Waiting for network to be connected");
		}
		else
		{
			LOG_INF("Network disconnected");
			connected = false;
			// Disconnect s MQTT brokera ako je odspojen s interneta
			(void)mqtt_helper_disconnect();
		}
		k_sem_reset(&run_app);
		return;
	}
}

// Definiranje funkcije za subscribe na topics
static void subscribe(void)
{
	int err;

	// Deklaracija varijable type mqtt_topic
	struct mqtt_topic subscribe_topic = {
		.topic = {
			.utf8 = CONFIG_MQTT_SAMPLE_SUB_TOPIC,
			.size = strlen(CONFIG_MQTT_SAMPLE_SUB_TOPIC)},
		.qos = MQTT_QOS_1_AT_LEAST_ONCE};

	// Definiranje subscription liste
	struct mqtt_subscription_list subscription_list = {
		.list = &subscribe_topic,
		.list_count = 1,
		.message_id = SUBSCRIBE_TOPIC_ID};

	// Subscribe na topics
	LOG_INF("Subscribing to %s", CONFIG_MQTT_SAMPLE_SUB_TOPIC);
	err = mqtt_helper_subscribe(&subscription_list);
	if (err)
	{
		LOG_ERR("Failed to subscribe to topics, error: %d", err);
		return;
	}
}

// Definiranje funkcije za publish podataka
static int publish(uint8_t *data, size_t len)
{
	int err;
	// Deklaracija varijabli za type mqtt_publish_param
	struct mqtt_publish_param mqtt_param;

	mqtt_param.message.payload.data = data;
	mqtt_param.message.payload.len = len;
	mqtt_param.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
	mqtt_param.message_id = mqtt_helper_msg_id_get();
	mqtt_param.message.topic.topic.utf8 = CONFIG_MQTT_SAMPLE_PUB_TOPIC;
	mqtt_param.message.topic.topic.size = strlen(CONFIG_MQTT_SAMPLE_PUB_TOPIC);
	mqtt_param.dup_flag = 0;
	mqtt_param.retain_flag = 0;

	// Publish na MQTT broker
	err = mqtt_helper_publish(&mqtt_param);
	if (err)
	{
		LOG_WRN("Failed to send payload, err: %d", err);
		return err;
	}

	LOG_INF("Published message: \"%.*s\" on topic: \"%.*s\"", mqtt_param.message.payload.len,
			mqtt_param.message.payload.data,
			mqtt_param.message.topic.topic.size,
			mqtt_param.message.topic.topic.utf8);
	return 0;
}

// Definiranje callback handlera za CONNACK event
static void on_mqtt_connack(enum mqtt_conn_return_code return_code, bool session_present)
{
	if (return_code == MQTT_CONNECTION_ACCEPTED)
	{
		LOG_INF("Connected to MQTT broker");
		LOG_INF("Hostname: %s", CONFIG_MQTT_SAMPLE_BROKER_HOSTNAME);
		LOG_INF("Client ID: %s", (char *)client_id);
		LOG_INF("Port: %d", CONFIG_MQTT_HELPER_PORT);
		LOG_INF("TLS: %s", IS_ENABLED(CONFIG_MQTT_LIB_TLS) ? "Yes" : "No");
		subscribe();
	}
	else
	{
		LOG_WRN("Connection to broker not established, return_code: %d", return_code);
	}

	ARG_UNUSED(return_code);
}

// Define callback handlera za SUBACK event
static void on_mqtt_suback(uint16_t message_id, int result)
{
	if (result != MQTT_SUBACK_FAILURE)
	{
		if (message_id == SUBSCRIBE_TOPIC_ID)
		{
			LOG_INF("Subscribed to %s with QoS %d", CONFIG_MQTT_SAMPLE_SUB_TOPIC, result);
			return;
		}
		LOG_WRN("Subscribed to unknown topic, id: %d with QoS %d", message_id, result);
		return;
	}
	LOG_ERR("Topic subscription failed, error: %d", result);
}

// Definiranje callback handlera za PUBLISH event
static void on_mqtt_publish(struct mqtt_helper_buf topic, struct mqtt_helper_buf payload)
{
	LOG_INF("Received payload: %.*s on topic: %.*s", payload.size,
			payload.ptr,
			topic.size,
			topic.ptr);

	if (strncmp(payload.ptr, LED1_ON_CMD,
				sizeof(LED1_ON_CMD) - 1) == 0)
	{
		dk_set_led_on(DK_LED1);
	}
	else if (strncmp(payload.ptr, LED1_OFF_CMD, sizeof(LED1_OFF_CMD) - 1) == 0)
	{
		dk_set_led_off(DK_LED1);
	}
	else if (strncmp(payload.ptr, LED2_ON_CMD, sizeof(LED2_ON_CMD) - 1) == 0)
	{
		dk_set_led_on(DK_LED2);
	}
	else if (strncmp(payload.ptr, LED2_OFF_CMD, sizeof(LED2_OFF_CMD) - 1) == 0)
	{
		dk_set_led_off(DK_LED2);
	}
}

// Definiranje callback handlera za DISCONNECT event
static void on_mqtt_disconnect(int result)
{
	LOG_INF("MQTT client disconnected: %d", result);
}

#if defined(CONFIG_MQTT_LIB_TLS)
// Funkcija za registraciju TLS certifikata
static int tls_credentials_provision(void)
{
	int err;

	LOG_INF("Provisioning TLS credentials...");

	err = tls_credential_add(TLS_SEC_TAG, TLS_CREDENTIAL_CA_CERTIFICATE,
				 ca_certificate, sizeof(ca_certificate));
	if (err == -EEXIST) {
		LOG_INF("CA certificate already exists, skipping");
		return 0;
	} else if (err < 0) {
		LOG_ERR("Failed to add CA certificate: %d", err);
		return err;
	}

	LOG_INF("TLS credentials provisioned successfully");
	return 0;
}
#endif

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	if (has_changed & DK_BTN1_MSK && button_state & DK_BTN1_MSK)
	{
		// Publish message ako je pritisnuta tipka 1
		int err = publish(BUTTON1_MSG, sizeof(BUTTON1_MSG) - 1);
		if (err)
		{
			LOG_ERR("Failed to send message, %d", err);
			return;
		}
	}
	else if (has_changed & DK_BTN2_MSK && button_state & DK_BTN2_MSK)
	{
		// Publish message ako je pritisnuta tipka 2
		int err = publish(BUTTON2_MSG, sizeof(BUTTON2_MSG) - 1);
		if (err)
		{
			LOG_ERR("Failed to send message, %d", err);
			return;
		}
	}
}

/* ---------- global state ---------- */
static lv_obj_t *g_scr1;
static lv_obj_t *g_scr2;
static lv_obj_t *g_lbl_steps;
static lv_obj_t *g_lbl_distance;
static lv_obj_t *g_lbl_calories;
static lv_obj_t *g_lbl_activity;
static lv_obj_t *g_lbl_speed;

// static bool g_on_second;
static struct k_work g_toggle_work;

/* ---------- GPIO descriptors ---------- */
static const struct gpio_dt_spec sw1 =
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0});
static struct gpio_callback sw1_cb;

/* ---- User-config (runtime) ---- */
typedef struct
{
	float height_cm;			   // visina
	float step_length_m;		   // duljina koraka (m)
	float weight_kg;			   // težina
	float kcal_per_step;		   // kcal po koraku
	float dyn_k;				   // za dinamički prag μ + k·σ
	float motion_std_min;		   // gate na σ
	uint32_t min_step_interval_ms; // minimalni razmak između koraka
	uint32_t startup_grace_ms;	   // grace nakon boota/reset
} user_cfg_t;

/* inicijalne vrijednosti */
static user_cfg_t g_cfg = {
	.height_cm = 175.0f,
	.step_length_m = 0.7f,
	.weight_kg = 70.0f,
	.kcal_per_step = 0.04f,
	.dyn_k = 1.5f,
	.motion_std_min = 0.04f,
	.min_step_interval_ms = 500,
	.startup_grace_ms = 1500,
};

static float clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

static float step_length_from_cadence(float cadence_hz, float height_cm, bool running)
{
	// baza iz visine
	float base = clampf(0.413f * (height_cm / 100.0f), 0.35f, 0.90f);

	// model iz kadence
	float L_model = running ? (0.60f + 0.35f * cadence_hz)	// trčanje
							: (0.35f + 0.25f * cadence_hz); // hodanje
	float L = clampf(L_model, 0.45f, 1.50f);

	// zbog stabilnosti i obranjivosti
	return 0.5f * base + 0.5f * L;
}

static void derive_params_from_hw(void)
{
	// duljina koraka (hod): 0.413 × visina
	float h_m = g_cfg.height_cm / 100.0f;
	g_cfg.step_length_m = clampf(0.413f * h_m, 0.35f, 0.90f);

	// kcal/korak – gruba ali praktična aproksimacija
	g_cfg.kcal_per_step = 0.04f * (g_cfg.weight_kg / 70.0f);

	// ostalo – defaulti
	if (g_cfg.dyn_k <= 0.0f)
		g_cfg.dyn_k = 1.5f;
	if (g_cfg.motion_std_min <= 0.0f)
		g_cfg.motion_std_min = 0.04f;
	if (g_cfg.min_step_interval_ms == 0)
		g_cfg.min_step_interval_ms = 400; // hvata hod bolje
	if (g_cfg.startup_grace_ms == 0)
		g_cfg.startup_grace_ms = 1500;
}

/* ---- LVGL widgets za Settings ---- */
static lv_obj_t *sp_step_len_cm;	 // spinbox (cm)
static lv_obj_t *sp_weight_kg;		 // spinbox (kg)
static lv_obj_t *sp_kcal_step_x1000; // spinbox (kcal/korak *1000)
static lv_obj_t *sl_dyn_k;			 // slider (1.0–3.0) *100
static lv_obj_t *sl_motion_std;		 // slider (0.01–0.15) *1000
static lv_obj_t *sp_min_step_ms;	 // spinbox (ms)
static lv_obj_t *sp_startup_ms;		 // spinbox (ms)
static lv_obj_t *lbl_dyn_k_val;		 // value labels uz slidere
static lv_obj_t *lbl_motion_std_val;

/* ============================================================= */
/*  LVGL / GPIO  CALLBACKS                                        */
/* ============================================================= */

// Funkcija za ažuriranje prikaza
static void update_step_display(int steps, float distance_m, float calories, const char *activity, float speed_m_s)
{
	char buf[64];

	// Ažuriraj steps
	snprintf(buf, sizeof(buf), LV_SYMBOL_PLAY "  Steps: %d", steps);
	lv_label_set_text(g_lbl_steps, buf);

	// Ažuriraj distance
	snprintf(buf, sizeof(buf), LV_SYMBOL_GPS "  Distance: %.2f m", (double)(distance_m));
	lv_label_set_text(g_lbl_distance, buf);

	// Ažuriraj calories
	snprintf(buf, sizeof(buf), LV_SYMBOL_BATTERY_FULL "  Calories: %.2f kcal", (double)(calories));
	lv_label_set_text(g_lbl_calories, buf);

	// Ažuriraj activity
	snprintf(buf, sizeof(buf), LV_SYMBOL_SETTINGS "  Activity: %s", activity);
	lv_label_set_text(g_lbl_activity, buf);

	// Ažuriraj speed
	snprintf(buf, sizeof(buf), LV_SYMBOL_CHARGE "  Speed: %.2f m/s", (double)(speed_m_s));
	lv_label_set_text(g_lbl_speed, buf);
}

static void sw1_isr_toggle(const struct device *dev,
						   struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	k_work_submit(&g_toggle_work);
}

static void toggle_work_fn(struct k_work *w)
{
	lv_obj_t *cur = lv_disp_get_scr_act(NULL);
	lv_obj_t *next = (cur == g_scr1) ? g_scr2 : g_scr1;
	lv_scr_load_anim(next, (cur == g_scr1) ? LV_SCR_LOAD_ANIM_MOVE_LEFT : LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
}

static void btn_to_scr2_cb(lv_event_t *e)
{
	ARG_UNUSED(e);

	lv_scr_load_anim(g_scr2, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
}

static void btn_left_cb(lv_event_t *e)
{
	ARG_UNUSED(e);
	// Provjera jesmo li trenutno na drugom ekranu
	lv_obj_t *current_screen = lv_disp_get_scr_act(NULL);

	if (current_screen == g_scr2)
	{
		// Animacija za prelazak na prvi ekran
		lv_scr_load_anim(g_scr1, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
		LOG_INF("Switching to Screen 1");
	}
}

static void btn_right_cb(lv_event_t *e)
{
	ARG_UNUSED(e);

	steps = 0;
	distance_m = 0;
	calories = 0;
	speed_m_s = 0;
	activity = "Standing";
	step_detected = false;
	last_step_time = k_uptime_get();
	reset_time = 0;
	// previous_total_acc = 0.0f;
	previous_hp = 0.0f;
	is_running = false;
	memset(acc_buf, 0, sizeof(acc_buf));
	acc_sum = 0.0f;
	acc_idx = 0;
	memset(std_buf, 0, sizeof(std_buf));
	std_sum = 0.0f;
	std_sq_sum = 0.0f;
	std_idx = 0;
	sample_count = 0;
	cadence_hz_ema = 0.0f;

	char msg[MESSAGE_BUFFER_SIZE];
	int len = snprintk(msg, sizeof(msg), "Data reset!");
	int err = publish((uint8_t *)msg, len);
	if (err)
	{
		LOG_WRN("MQTT publish failed: %d", err);
	}
	else
	{
		LOG_INF("Published: %s", msg);
	}
	// Ažuriraj prikaz
	update_step_display(0, 0, 0, "-", 0.0f);
}

/* ============================================================= */
/*  INITIALISATION HELPERS                                        */
/* ============================================================= */

static bool init_display(const struct device **disp_out)
{
	const struct device *disp =
		DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (!device_is_ready(disp))
	{
		LOG_ERR("Display not ready");
		return false;
	}
	*disp_out = disp;
	return true;
}

static void init_buttons(void)
{
	if (gpio_is_ready_dt(&sw1))
	{
		gpio_pin_configure_dt(&sw1, GPIO_INPUT);
		gpio_init_callback(&sw1_cb, sw1_isr_toggle, BIT(sw1.pin));
		gpio_add_callback(sw1.port, &sw1_cb);
		gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_EDGE_TO_ACTIVE);
	}
}

/* --- Helpers za spinbox/slider --- */
static lv_obj_t *make_label(lv_obj_t *parent, const char *txt, lv_align_t align, int x_ofs, int y_ofs)
{
	lv_obj_t *l = lv_label_create(parent);
	if (!l)
	{
		LOG_ERR("lv_label_create failed");
		return NULL;
	}
	lv_label_set_text(l, txt);
	lv_obj_align(l, align, x_ofs, y_ofs);
	return l;
}

static lv_obj_t *make_spin_int(lv_obj_t *parent, int32_t min, int32_t max, int32_t step, int32_t init, lv_align_t align, int x_ofs, int y_ofs)
{
	lv_obj_t *sb = lv_spinbox_create(parent);
	if (!sb)
	{
		LOG_ERR("lv_spinbox_create failed");
		return NULL;
	}
	lv_spinbox_set_range(sb, min, max);
	lv_spinbox_set_digit_format(sb, 6, 0);
	lv_spinbox_set_step(sb, step);
	lv_spinbox_set_value(sb, init);
	lv_obj_set_width(sb, 90);
	lv_obj_align(sb, align, x_ofs, y_ofs);
	return sb;
}

static lv_obj_t *make_slider(lv_obj_t *parent, int32_t min, int32_t max, int32_t init, lv_align_t align, int x_ofs, int y_ofs)
{
	lv_obj_t *s = lv_slider_create(parent);
	if (!s)
	{
		LOG_ERR("lv_slider_create failed");
		return NULL;
	}
	lv_slider_set_range(s, min, max);
	lv_slider_set_value(s, init, LV_ANIM_OFF);
	lv_obj_set_width(s, 140);
	lv_obj_align(s, align, x_ofs, y_ofs);
	return s;
}

/* --- Slider live labels --- */
static void dyn_k_slider_cb(lv_event_t *e)
{
	if (!sl_dyn_k || !lbl_dyn_k_val)
		return; // <--- guard
	int v = lv_slider_get_value(sl_dyn_k);
	float k = v / 100.0f;
	char buf[16];
	snprintf(buf, sizeof(buf), "k=%.2f", (double)k);
	lv_label_set_text(lbl_dyn_k_val, buf);
}

static void motion_std_slider_cb(lv_event_t *e)
{
	if (!sl_motion_std || !lbl_motion_std_val)
		return; // <--- guard
	int v = lv_slider_get_value(sl_motion_std);
	float s = v / 1000.0f;
	char buf[16];
	snprintf(buf, sizeof(buf), "σ>=%.3f", (double)s);
	lv_label_set_text(lbl_motion_std_val, buf);
}

/* --- APPLY: učitaj polja i primijeni u g_cfg + resetiraj state/filtre --- */
static void apply_settings_cb(lv_event_t *e)
{
	int32_t h_cm = lv_spinbox_get_value(sp_step_len_cm); // Height (cm)
	int32_t kg = lv_spinbox_get_value(sp_weight_kg);	 // Weight (kg)

	g_cfg.height_cm = (float)h_cm;
	g_cfg.weight_kg = (float)kg;

	derive_params_from_hw(); // izračunava step_length_m, kcal_per_step, itd.

	// reset state + filtri
	steps = 0;
	distance_m = 0;
	calories = 0;
	speed_m_s = 0;
	activity = "Standing";
	step_detected = false;
	is_running = false;
	last_step_time = k_uptime_get();
	reset_time = 0;
	previous_hp = 0.0f;
	memset(acc_buf, 0, sizeof(acc_buf));
	acc_sum = 0.0f;
	acc_idx = 0;
	memset(std_buf, 0, sizeof(std_buf));
	std_sum = 0.0f;
	std_sq_sum = 0.0f;
	std_idx = 0;
	filters_ready = false;
	sample_count = 0;
	cadence_hz_ema = 0.0f;

	update_step_display(steps, distance_m, calories, "-", speed_m_s);

	char msg[MESSAGE_BUFFER_SIZE];
	int len = snprintk(msg, sizeof(msg),
					   "Applied H/W: H=%.0fcm W=%.0fkg -> step=%.2fm, kcal/step=%.3f, k=%.2f, sigma>=%.3f, min=%dms",
					   (double)g_cfg.height_cm, (double)g_cfg.weight_kg,
					   (double)g_cfg.step_length_m, (double)g_cfg.kcal_per_step,
					   (double)g_cfg.dyn_k, (double)g_cfg.motion_std_min,
					   (int)g_cfg.min_step_interval_ms);
	if (len > 0)
		(void)publish((uint8_t *)msg, (size_t)len);
}

/* Compact row helper */
static lv_obj_t *make_row(lv_obj_t *parent)
{
	lv_obj_t *row = lv_obj_create(parent);
	if (!row)
	{
		LOG_ERR("lv_obj_create(row) failed");
		return NULL;
	}
	lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
	lv_obj_set_style_bg_opa(row, LV_OPA_0, 0);
	lv_obj_set_style_border_width(row, 0, 0);
	lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
	lv_obj_set_style_pad_row(row, 0, 0);
	lv_obj_set_style_pad_column(row, 6, 0);
	return row;
}

/* + / - callbacks za spinbox */
static void spin_inc_cb(lv_event_t *e)
{
	lv_obj_t *spin = lv_event_get_user_data(e);
	lv_spinbox_increment(spin);
}
static void spin_dec_cb(lv_event_t *e)
{
	lv_obj_t *spin = lv_event_get_user_data(e);
	lv_spinbox_decrement(spin);
}

/*  + i - gumbi sa spinboxom */
void add_stepper_buttons(lv_obj_t *parent, lv_obj_t *spin)
{
	/* gumbi kao djeca istog parenta kao i spinbox */
	lv_obj_t *p = lv_obj_get_parent(spin);

	lv_obj_t *btn_minus = lv_btn_create(p);
	lv_obj_set_size(btn_minus, 26, 26);
	lv_obj_align_to(btn_minus, spin, LV_ALIGN_OUT_LEFT_MID, -6, 0);
	lv_obj_add_event_cb(btn_minus, spin_dec_cb, LV_EVENT_CLICKED, spin);
	lv_label_set_text(lv_label_create(btn_minus), LV_SYMBOL_MINUS);

	lv_obj_t *btn_plus = lv_btn_create(p);
	lv_obj_set_size(btn_plus, 26, 26);
	lv_obj_align_to(btn_plus, spin, LV_ALIGN_OUT_RIGHT_MID, 6, 0);
	lv_obj_add_event_cb(btn_plus, spin_inc_cb, LV_EVENT_CLICKED, spin);
	lv_label_set_text(lv_label_create(btn_plus), LV_SYMBOL_PLUS);

	lv_obj_move_foreground(btn_minus);
	lv_obj_move_foreground(btn_plus);
}

static void create_screens(void)
{
	/* --- Screen 1 --- */
	g_scr1 = lv_scr_act(); /* default screen */
	lv_obj_set_style_bg_color(g_scr1, lv_color_hex(0x303080), 0);

	// Glavni naslov
	lv_obj_t *lbl_hello = lv_label_create(g_scr1);
	lv_label_set_text(lbl_hello, LV_SYMBOL_SHUFFLE "  Step Counter");
	lv_obj_align(lbl_hello, LV_ALIGN_TOP_MID, 0, 20);
	lv_obj_set_style_text_color(lbl_hello, lv_color_white(), LV_PART_MAIN);
	lv_obj_set_style_text_font(lbl_hello, &lv_font_montserrat_14, LV_PART_MAIN);

	// Label za korake
	g_lbl_steps = lv_label_create(g_scr1);
	lv_label_set_text(g_lbl_steps, LV_SYMBOL_PLAY "  Steps: 0");
	lv_obj_align(g_lbl_steps, LV_ALIGN_CENTER, 0, -40);
	lv_obj_set_style_text_color(g_lbl_steps, lv_color_hex(0x4ade80), LV_PART_MAIN);
	lv_obj_set_style_text_font(g_lbl_steps, &lv_font_montserrat_14, LV_PART_MAIN);

	// Label za udaljenost
	g_lbl_distance = lv_label_create(g_scr1);
	lv_label_set_text(g_lbl_distance, LV_SYMBOL_GPS "  Distance: 0 m");
	lv_obj_align(g_lbl_distance, LV_ALIGN_CENTER, 0, -20);
	lv_obj_set_style_text_color(g_lbl_distance, lv_color_hex(0x60a5fa), LV_PART_MAIN);
	lv_obj_set_style_text_font(g_lbl_distance, &lv_font_montserrat_14, LV_PART_MAIN);

	// Label za kalorije
	g_lbl_calories = lv_label_create(g_scr1);
	lv_label_set_text(g_lbl_calories, LV_SYMBOL_BATTERY_FULL "  Calories: 0 kcal");
	lv_obj_align(g_lbl_calories, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_text_color(g_lbl_calories, lv_color_hex(0xf97316), LV_PART_MAIN);
	lv_obj_set_style_text_font(g_lbl_calories, &lv_font_montserrat_14, LV_PART_MAIN);

	// Label za aktivnost
	g_lbl_activity = lv_label_create(g_scr1);
	lv_label_set_text(g_lbl_activity, LV_SYMBOL_SETTINGS "  Activity: -");
	lv_obj_align(g_lbl_activity, LV_ALIGN_CENTER, 0, 20);
	lv_obj_set_style_text_color(g_lbl_activity, lv_color_hex(0xec4899), LV_PART_MAIN);
	lv_obj_set_style_text_font(g_lbl_activity, &lv_font_montserrat_14, LV_PART_MAIN);

	// Label za brzinu
	g_lbl_speed = lv_label_create(g_scr1);
	lv_label_set_text(g_lbl_speed, LV_SYMBOL_CHARGE "  Speed: 0 m/s");
	lv_obj_align(g_lbl_speed, LV_ALIGN_CENTER, 0, 40);
	lv_obj_set_style_text_color(g_lbl_speed, lv_color_hex(0xec4899), LV_PART_MAIN);
	lv_obj_set_style_text_font(g_lbl_speed, &lv_font_montserrat_14, LV_PART_MAIN);

	/* --- GO TO SCREEN 2 BUTTON --- */
	lv_obj_t *btn_to2 = lv_btn_create(g_scr1);
	lv_obj_set_size(btn_to2, 100, 40);
	lv_obj_align(btn_to2, LV_ALIGN_CENTER, 0, 80);
	lv_obj_add_event_cb(btn_to2, btn_to_scr2_cb, LV_EVENT_CLICKED, NULL);
	lv_label_set_text(lv_label_create(btn_to2), "DETAILS");

	/* --- Screen 2 --- */
	g_scr2 = lv_obj_create(NULL);
	lv_obj_set_style_bg_color(g_scr2, lv_color_hex(0x303080), 0);

	lv_obj_t *lbl2 = lv_label_create(g_scr2);
	lv_label_set_text(lbl2, "Details");
	lv_obj_set_style_text_color(lbl2, lv_color_white(), LV_PART_MAIN);
	lv_obj_center(lbl2);

	/* --- SETTINGS PANEL (Height/Weight only) --- */

	/* Height (cm) */
	lv_obj_t *lbl = lv_label_create(g_scr2);
	lv_label_set_text(lbl, "Height (cm)");
	lv_obj_align(lbl, LV_ALIGN_CENTER, 0, -90); // label centar

	sp_step_len_cm = make_spin_int(
		g_scr2, 120, 220, 1, (int)g_cfg.height_cm,
		LV_ALIGN_CENTER, 0, -60 // spinbox centar
	);
	if (sp_step_len_cm)
	{
		lv_obj_set_width(sp_step_len_cm, 80);
		add_stepper_buttons(g_scr2, sp_step_len_cm); // +/- uz spinbox
	}

	/* Weight (kg) */
	lbl = lv_label_create(g_scr2);
	lv_label_set_text(lbl, "Weight (kg)");
	lv_obj_align(lbl, LV_ALIGN_CENTER, 0, -30); // drugi label centar

	sp_weight_kg = make_spin_int(
		g_scr2, 30, 200, 1, (int)g_cfg.weight_kg,
		LV_ALIGN_CENTER, 0, 0 // spinbox centar
	);
	if (sp_weight_kg)
	{
		lv_obj_set_width(sp_weight_kg, 80);
		add_stepper_buttons(g_scr2, sp_weight_kg);
	}

	/* APPLY */
	lv_obj_t *btn_apply = lv_btn_create(g_scr2);
	lv_obj_set_size(btn_apply, 70, 30);
	lv_obj_align(btn_apply, LV_ALIGN_CENTER, 0, 100);
	lv_obj_add_event_cb(btn_apply, apply_settings_cb, LV_EVENT_CLICKED, NULL);
	lv_label_set_text(lv_label_create(btn_apply), "APPLY");

	/* BACK */
	lv_obj_t *btn_left = lv_btn_create(g_scr2);
	lv_obj_set_size(btn_left, 70, 30);
	lv_obj_align(btn_left, LV_ALIGN_CENTER, -40, 60);
	lv_obj_add_event_cb(btn_left, btn_left_cb, LV_EVENT_CLICKED, NULL);
	lv_label_set_text(lv_label_create(btn_left), "BACK");

	/* RESET */
	lv_obj_t *btn_right = lv_btn_create(g_scr2);
	lv_obj_set_size(btn_right, 70, 30);
	lv_obj_align(btn_right, LV_ALIGN_CENTER, 40, 60);
	lv_obj_add_event_cb(btn_right, btn_right_cb, LV_EVENT_CLICKED, NULL);
	lv_label_set_text(lv_label_create(btn_right), "RESET");
}

/* ============================================================= */
/*  MAIN                                                          */
/* ============================================================= */

void main(void)
{
	int16_t buf0, buf1, buf2;
	struct adc_sequence sequence0 = {.buffer = &buf0, .buffer_size = sizeof(buf0)};
	struct adc_sequence sequence1 = {.buffer = &buf1, .buffer_size = sizeof(buf1)};
	struct adc_sequence sequence2 = {.buffer = &buf2, .buffer_size = sizeof(buf2)};

	if (!adc_is_ready_dt(&adc_channel0) || adc_channel_setup_dt(&adc_channel0) < 0 || adc_sequence_init_dt(&adc_channel0, &sequence0) < 0 ||
		!adc_is_ready_dt(&adc_channel1) || adc_channel_setup_dt(&adc_channel1) < 0 || adc_sequence_init_dt(&adc_channel1, &sequence1) < 0 ||
		!adc_is_ready_dt(&adc_channel2) || adc_channel_setup_dt(&adc_channel2) < 0 || adc_sequence_init_dt(&adc_channel2, &sequence2) < 0)
	{
		LOG_ERR("ADC setup failed");
		return;
	}

	last_step_time = k_uptime_get(); // da cadence_ok ne prođe odmah na startu
	int err;
	if (dk_leds_init() != 0)
	{
		LOG_ERR("Failed to initialize the LED library");
	}
	// Sleep za omogućavanje Wi-Fi drivera
	k_sleep(K_SECONDS(1));

	net_mgmt_init_event_callback(&mgmt_cb, net_mgmt_event_handler, EVENT_MASK);
	net_mgmt_add_event_callback(&mgmt_cb);

	LOG_INF("Waiting to connect to Wi-Fi");
	k_sem_take(&run_app, K_FOREVER);

	if (dk_buttons_init(button_handler) != 0)
	{
		LOG_ERR("Failed to initialize the buttons library");
	}

#if defined(CONFIG_MQTT_LIB_TLS)
	// Postavi TLS certifikate prije MQTT inicijalizacije
	err = tls_credentials_provision();
	if (err)
	{
		LOG_ERR("Failed to provision TLS credentials, error: %d", err);
		return;
	}
#endif

	// Inicijalizacija MQTT helper knjižnice
	struct mqtt_helper_cfg config = {
		.cb = {
			.on_connack = on_mqtt_connack,
			.on_disconnect = on_mqtt_disconnect,
			.on_publish = on_mqtt_publish,
			.on_suback = on_mqtt_suback,
		},
	};

	err = mqtt_helper_init(&config);
	if (err)
	{
		LOG_ERR("Failed to initialize MQTT helper, error: %d", err);
		return;
	}

	// Generiranje klijent ID-a
	uint32_t id = sys_rand32_get();
	snprintf(client_id, sizeof(client_id), "%s-%010u", CONFIG_BOARD, id);

	// Uspostava konekcije s MQTT brokerom
	struct mqtt_helper_conn_params conn_params = {
		.hostname.ptr = CONFIG_MQTT_SAMPLE_BROKER_HOSTNAME,
		.hostname.size = strlen(CONFIG_MQTT_SAMPLE_BROKER_HOSTNAME),
		.device_id.ptr = (char *)client_id,
		.device_id.size = strlen(client_id),
	};

	err = mqtt_helper_connect(&conn_params);
	if (err)
	{
		LOG_ERR("Failed to connect to MQTT, error code: %d", err);
		return;
	}

	const struct device *disp;

	if (!init_display(&disp))
		return;

	k_work_init(&g_toggle_work, toggle_work_fn);
	init_buttons();
	create_screens();
	derive_params_from_hw();

	lv_timer_handler();
	display_blanking_off(disp);

	while (1)
	{
		if (adc_read(adc_channel0.dev, &sequence0) < 0 ||
			adc_read(adc_channel1.dev, &sequence1) < 0 ||
			adc_read(adc_channel2.dev, &sequence2) < 0)
		{
			LOG_ERR("ADC read failed");
			continue;
		}

		int val_mv0 = buf0, val_mv1 = buf1, val_mv2 = buf2;
		if (adc_raw_to_millivolts_dt(&adc_channel0, &val_mv0) < 0 ||
			adc_raw_to_millivolts_dt(&adc_channel1, &val_mv1) < 0 ||
			adc_raw_to_millivolts_dt(&adc_channel2, &val_mv2) < 0)
		{
			LOG_WRN("mV conversion failed");
			continue;
		}

		float x_g = (val_mv0 - ADC_OFFSET_MV) / ADC_SCALE_MV;
		float y_g = (val_mv1 - ADC_OFFSET_MV) / ADC_SCALE_MV;
		float z_g = (val_mv2 - ADC_OFFSET_MV) / ADC_SCALE_MV;

		float total_acc = sqrtf(x_g * x_g + y_g * y_g + z_g * z_g);

		// --- PRVI UZORCI ---
		if (!filters_ready)
		{
			// 1) Prvi prolaz
			if (sample_count == 0)
			{
				for (int i = 0; i < HP_WIN_SAMPLES; ++i)
				{
					acc_buf[i] = total_acc;
					acc_sum += total_acc;
				}
				acc_idx = 0;

				// σ prozor
				for (int i = 0; i < STD_WIN_SAMPLES; ++i)
				{
					std_buf[i] = 0.0f;
				}
				std_idx = 0;

				previous_hp = 0.0f;
				last_step_time = k_uptime_get();
			}

			sample_count++;

			// detekcija tek kad smo napunili prozore i prošla je kratka pauza
			if (sample_count >= WARMUP_SAMPLES)
			{
				if (k_uptime_get() - last_step_time >= g_cfg.startup_grace_ms)
				{
					filters_ready = true;
				}
			}
		}

		// --- HIGH-PASS preko kliznog prosjeka (za micanje gravitacije i sporih driftova) ---
		acc_sum -= acc_buf[acc_idx];
		acc_sum += total_acc;
		float mean = acc_sum / HP_WIN_SAMPLES;
		acc_buf[acc_idx] = total_acc;
		acc_idx = (acc_idx + 1) % HP_WIN_SAMPLES;

		float acc_hp = total_acc - mean; // dio za detekciju

		// --- Statistika za dinamički prag (μ i σ nad HP signalom) ---
		float hp = acc_hp;
		std_sum -= std_buf[std_idx];
		std_sq_sum -= std_buf[std_idx] * std_buf[std_idx];
		std_sum += hp;
		std_sq_sum += hp * hp;
		std_buf[std_idx] = hp;
		std_idx = (std_idx + 1) % STD_WIN_SAMPLES;

		float mu = std_sum / STD_WIN_SAMPLES;
		float var = fmaxf(0.0f, (std_sq_sum / STD_WIN_SAMPLES) - mu * mu);
		float sig = sqrtf(var);

		// "no-motion" gate: ako je σ premalen, ignoriramo sve jer vjerojatno stojimo ili je lažni signal
		bool motion_ok = (sig >= g_cfg.motion_std_min);

		// Dinamički prag
		float dyn_thr = mu + g_cfg.dyn_k * sig;

		// Rising-edge nad HP signalom i din. pragom
		int64_t now = k_uptime_get();
		bool rising_edge = (previous_hp <= dyn_thr) && (acc_hp > dyn_thr);
		previous_hp = acc_hp;

		if (!step_detected && motion_ok && rising_edge && (now - last_step_time > g_cfg.min_step_interval_ms))
		{
			int64_t step_time_ms = now - last_step_time;
			float step_time_s = step_time_ms / 1000.0f;

			// 1) Kadenca (instant + EMA)
			float cadence_hz_inst = 1.0f / fmaxf(step_time_s, 1e-3f);
			cadence_hz_ema = (1.0f - CADENCE_ALPHA) * cadence_hz_ema + CADENCE_ALPHA * cadence_hz_inst;
			float spm = cadence_hz_ema * 60.0f;

			// 2) Adaptivna duljina koraka
			float step_len_m = step_length_from_cadence(cadence_hz_ema, g_cfg.height_cm, is_running);

			// 3) Brzina iz adaptivne duljine
			speed_m_s = step_len_m / step_time_s;

			// 4) Klasifikacija
			bool suggest_run = (spm >= RUN_SPM_HI) || (speed_m_s >= RUN_SPEED_HI);
			bool suggest_walk = (spm <= RUN_SPM_LO) && (speed_m_s <= RUN_SPEED_LO);

			if (suggest_run)
				is_running = true;
			else if (suggest_walk)
				is_running = false;

			step_len_m = step_length_from_cadence(cadence_hz_ema, g_cfg.height_cm, is_running);

			// 5) Ažuriranja metrika
			steps++;
			distance_m += step_len_m;
			calories = steps * g_cfg.kcal_per_step;
			last_step_time = now;
			reset_time = now + STEP_RESET_WINDOW_MS;
			step_detected = true;

			activity = is_running ? "Running" : "Walking";

			// MQTT Poruka
			char msg[MESSAGE_BUFFER_SIZE];
			int len = snprintk(msg, sizeof(msg), "Step detected! Total: %d, Distance: %.2f m, Calories: %.2f kcal, Speed: %.2f m/s, Activity: %s", (int)(steps), (float)(distance_m), (float)(calories), (float)(speed_m_s), activity);

			// Ažurira prikaz na ekranu
			update_step_display(steps, (double)(distance_m), (double)(calories), activity, (double)(speed_m_s));

			LOG_INF("Step detected! Total: %d, Distance: %.2f m, Calories: %.2f kcal, Speed: %.2f m/s, Activity: %s", steps, (float)(distance_m), (float)(calories), (float)(speed_m_s), activity);

			int err = publish((uint8_t *)msg, len);
			if (err)
			{
				LOG_WRN("MQTT publish failed: %d", err);
			}
			else
			{
				LOG_INF("Published: %s", msg);
			}
		}
		else if (step_detected && now >= reset_time)
		{
			step_detected = false;
		}

		lv_timer_handler();

		k_sleep(K_MSEC(40));
	}
}