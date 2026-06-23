#define LILYGO_T_SIM7670G_S3
#include "utilities.h"

#define TINY_GSM_MODEM_SIM7670G
#define TINY_GSM_RX_BUFFER 1024

#include <TinyGsmClient.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Preferences.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"

Preferences prefs;   // registro veicoli in NVS (sopravvive allo spegnimento)

// ===================== CONFIGURAZIONE =====================
#define APN              ""
#define TELEGRAM_TOKEN   ""
#define TELEGRAM_CHAT_ID ""
#define TWILIO_SID       ""
#define TWILIO_AUTH      ""
#define TWILIO_FROM      ""
#define CALL_TO          ""
#define NTFY_TOPIC       ""
#define GEOAPIFY_KEY     ""

#define I2C_SDA          2
#define I2C_SCL          1
#define MPU_INT_PIN      6   // GPIO6 = INT MPU-6050 -> EXT1 wakeup

// Soglie di default per i 3 livelli, in magnitudine CRESCENTE:
// tocca (lieve) < seduto (media) < muove/sposta (forte). Calibra le sovrascrive.
#define DEFAULT_SENS_TOCCO   2000   // livello 1: tocca, vibrazione lieve (la piu' bassa)
#define DEFAULT_SENS_SEDUTO  6000   // livello 2: qualcuno si siede, media
#define DEFAULT_SENS_MUOVE   15000  // livello 3: muove/sposta la moto, forte (la piu' alta)

#define POLL_ARMED       8000
#define POLL_DISARMED    20000
#define CALM_PERIOD      600000   // 10 min senza movimento -> fine allarme, torna in sleep
                                  // (NESSUN cap massimo: ogni evento MPU resetta questi 10 min)
#define CALL_REPEAT      60000    // richiama ogni minuto finche' il movimento continua
#define UPDATE_INTERVAL  10000    // 10s aggiornamento GPS live
#define LONG_CALM        600000   // 10 min fermo -> riattiva chiamate se erano sospese
#define SWITCH_AWAKE_MS  30000    // 30s sveglio dopo uno switch attivo/disattivo (ripensamento)
#define RESEND_MOVE_MIN  360      // 6h: reinvio "Stai spostando?" se ancora disarmato
#define BATTERY_MAH      3400
#define CHARGE_MA        500
#define STAY_AWAKE_MS    180000   // 3 min finestra comandi (primo boot / sveglia)
// Correzione batteria: AT+CBC misura dopo una caduta (~diodo) -> a 100% reali (4.20V)
// leggeva ~3.90V. Offset per compensare. Tara questo valore se serve piu' precisione.
#define BATT_CAL_OFFSET  0.30
// ==========================================================

TinyGsm modem(SerialAT);
MPU6050 mpu;

// ===== STATO PERSISTENTE in RTC: sopravvive al deep sleep =====
RTC_DATA_ATTR bool rtcArmed           = false;
RTC_DATA_ATTR long rtcThreshold       = DEFAULT_SENS_TOCCO;  // soglia di SCATTO = livello tocca scalato
RTC_DATA_ATTR long rtcSensMuove       = DEFAULT_SENS_MUOVE;  // livello 3: muove/sposta (la piu' alta)
RTC_DATA_ATTR long rtcSensSeduto      = DEFAULT_SENS_SEDUTO; // livello 2: seduto
RTC_DATA_ATTR long rtcSensTocco       = DEFAULT_SENS_TOCCO;  // livello 1: tocca (la piu' bassa)
RTC_DATA_ATTR int  rtcSensMult        = 100;                 // sensibilita' %: alta=130 default=100 bassa=80
RTC_DATA_ATTR bool rtcMutedCalls      = false;
RTC_DATA_ATTR bool rtcTheftMode       = false;
RTC_DATA_ATTR long rtcLastUpdId       = 0;
RTC_DATA_ATTR char rtcNtfyId[40]      = {0};
RTC_DATA_ATTR bool rtcFirstBoot       = true;
RTC_DATA_ATTR bool rtcCooldownMsgSent = false; // "Stai spostando?" gia' inviato (sessione movimento)
RTC_DATA_ATTR long rtcLastAlarmMsgId  = 0;      // ID messaggio allarme (persiste tra sleep)
RTC_DATA_ATTR long rtcLiveLocMsgId    = 0;      // ID live location GPS (persiste tra sleep)
RTC_DATA_ATTR long rtcAlarmPinId      = 0;      // pin LTE nativo (da cancellare all'arrivo GPS)
RTC_DATA_ATTR long rtcLteCapId        = 0;      // caption "zona LTE" sotto il pin (cancellata col pin)
RTC_DATA_ATTR long rtcGpsCapId        = 0;      // caption "mappa GPS" con freschezza (sotto live location)
RTC_DATA_ATTR int  rtcDisarmPhase     = 0;      // disarmato: 0=nessuno 1=ignora-1min 2=osserva-1min
RTC_DATA_ATTR long rtcMinSinceMoveMsg = 9999;   // minuti dall'ultimo "Stai spostando?" (reinvio a 6h)
RTC_DATA_ATTR float rtcBattV          = 0;      // batteria letta a freddo PRIMA del deep sleep (V)
// Stati attesa registro veicoli in RTC: sopravvivono se il dispositivo dorme a meta' flusso
RTC_DATA_ATTR bool rtcAwaitVehSave    = false;  // attendo: su quale veicolo salvo la calibrazione
RTC_DATA_ATTR bool rtcAwaitVehName    = false;  // attendo: nome del nuovo veicolo
RTC_DATA_ATTR bool rtcAwaitVehSelect  = false;  // attendo: quale veicolo caricare

// ===== STATO RUNTIME (si azzera ad ogni boot) =====
bool    gpsActive        = false;
float   lastLat = 0,    lastLon = 0;
int     lastSat = 0,    lastAcc = 0;
String  lastGoodPos      = "";
long    lastAlarmMsgId   = 0;
long    lastAlarmPinId   = 0;   // native location pin LTE (appross.)
bool    alerted          = false;
unsigned long lastMotion    = 0;
unsigned long lastAlarmUpd  = 0;
long    liveLocMsgId     = 0;
long    lastStatusMsgId  = 0;
long    lastPosMsgId     = 0;
long    trackMsgId       = 0;   // messaggio di STATO (modo, segnale, batteria)
long    trackLocId       = 0;   // UNICA live location (accuracy=raggio LTE / 0 GPS)
long    trackCapId       = 0;   // caption SOTTO la mappa (sorgente + ora)
unsigned long lastCall   = 0;   // ultimo squillo Twilio
int     trackMode        = 0;
unsigned long lastTrack      = 0;
unsigned long lastPoll       = 0;
unsigned long stayAwakeStart = 0;
bool          forceAwake     = false;
bool          justSwitched   = false;   // true dopo uno switch: finestra sveglia ridotta a 30s
int           maxLevelCalled = 0;       // livello max per cui ho gia' chiamato (escalation)
int           callsMade      = 0;       // n. chiamate fatte in questa sessione di allarme
unsigned long lastGpsFixMs   = 0;       // millis dell'ultimo fix GPS (per "ultimo agg. Xm fa")
bool          liveTracking   = false;   // "Posizione live" on-demand (tracking SENZA allarme)
unsigned long liveUntilMs    = 0;       // se >0: il live si auto-ferma a questo millis (es. /posizione = 10 min)
unsigned long awakeUntilMs   = 0;       // resta sveglio (in loop) fino a questo millis: i comandi lo estendono
// shorthand: gli stati di attesa veicolo vivono in RTC
#define awaitVehSave    rtcAwaitVehSave
#define awaitVehName    rtcAwaitVehName
#define awaitVehSelect  rtcAwaitVehSelect
// Orologio di rete letto a freddo dopo il boot (per i timestamp, senza desync HTTPS)
long          bootClockSec   = -1;      // secondi-del-giorno all'istante bootClockMs
unsigned long bootClockMs    = 0;
bool          modemReady     = false;   // modem acceso e connesso (per non leggere AT a vuoto)

// Shorthand leggibili
#define armed       rtcArmed
#define theftMode   rtcTheftMode
#define threshold   rtcThreshold
#define sensMuove   rtcSensMuove
#define sensSeduto  rtcSensSeduto
#define sensTocco   rtcSensTocco
#define sensMult    rtcSensMult
#define mutedCalls  rtcMutedCalls

// Soglie EFFETTIVE = valori calibrati scalati dal moltiplicatore di sensibilita'.
// alta=+30% (meno sensibile, anti-pioggia), default=100%, bassa=-20% (piu' sensibile).
static inline long effTocco()  { return rtcSensTocco  * rtcSensMult / 100; }  // livello 1 (scatto)
static inline long effSeduto() { return rtcSensSeduto * rtcSensMult / 100; }  // livello 2
static inline long effMuove()  { return rtcSensMuove  * rtcSensMult / 100; }  // livello 3

// Livello del movimento dal picco di magnitudine: 0=sotto soglia, 1=tocca, 2=seduto, 3=muove
int livelloMovimento(long peak) {
    if (peak >= effMuove())  return 3;
    if (peak >= effSeduto()) return 2;
    if (peak >= effTocco())  return 1;
    return 0;
}
String testoLivello(int lvl) {
    if (lvl >= 3) return "\xF0\x9F\x9A\x97 Stanno SPOSTANDO la moto!";
    if (lvl == 2) return "\xF0\x9F\x9A\xB2 Qualcuno si \xC3\xA8 seduto sulla moto!";
    return "\xF0\x9F\x91\x8A Stanno toccando la moto";
}

// ===================== UTILITY =====================
String urlEncode(String str) {
    String out = "";
    for (unsigned int i = 0; i < str.length(); i++) {
        char c = str.charAt(i);
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') out += c;
        else { char buf[4]; sprintf(buf, "%%%02X", (uint8_t)c); out += buf; }
    }
    return out;
}

String sendAT(const char* cmd, int waitMs = 800) {
    while (SerialAT.available()) SerialAT.read();
    SerialAT.print(cmd); SerialAT.print("\r\n");
    delay(waitMs);
    String resp = "";
    while (SerialAT.available()) resp += (char)SerialAT.read();
    resp.trim();
    Serial.print(">> "); Serial.print(cmd);
    Serial.print(" => "); Serial.println(resp);
    return resp;
}

bool checkRespond() {
    for (int j = 0; j < 10; j++) {
        SerialAT.print("AT\r\n");
        String r = SerialAT.readString();
        if (r.indexOf("OK") >= 0) return true;
        delay(200);
    }
    return false;
}

// ===================== TASTIERE TELEGRAM =====================
String tastiera() {
    return "{\"keyboard\":[["
           "\"\xF0\x9F\x94\xB4\xF0\x9F\x94\x92 Attiva\",\"\xF0\x9F\x9F\xA2\xF0\x9F\x94\x93 Disattiva\"],["
           "\"\xF0\x9F\x9A\xA8 Modalita furto\"],["
           "\"\xF0\x9F\x94\x95 Stop chiamate\",\"\xF0\x9F\x94\x94 Riattiva chiamate\"],["
           "\"\xF0\x9F\x93\x8D Posizione\",\"\xE2\x84\xB9 Stato\",\"\xF0\x9F\x8E\x9A Calibra\"],["
           "\"\xF0\x9F\x94\xBC Alta\",\"\xE2\x9E\x96 Default\",\"\xF0\x9F\x94\xBD Bassa\"],["
           "\"\xF0\x9F\x9A\x97 Seleziona veicolo\"]"
           "],\"resize_keyboard\":true}";
}

String tastieraFurto() {
    return "{\"keyboard\":[["
           "\"\xE2\x9A\xA1 Risparmio\",\"\xF0\x9F\x9B\xB0 GPS costante\"],["
           "\"\xE2\x9C\x96 Esci furto\"],["
           "\"\xF0\x9F\x93\x8D Posizione\",\"\xE2\x84\xB9 Stato\"]"
           "],\"resize_keyboard\":true}";
}

// ===================== TELEGRAM SEND =====================
// *** POST (non GET) ***: testo e tastiera vanno nel CORPO, non nell'URL.
// La tastiera urlencoded e' lunga ~800 byte: in URL superava il limite del
// comando AT del modem e l'invio falliva (per questo i bottoni non comparivano).
long tgSend(String testo, bool conTastiera = false) {
    String url = "https://api.telegram.org/bot" TELEGRAM_TOKEN "/sendMessage";
    String body = "chat_id=" TELEGRAM_CHAT_ID "&parse_mode=HTML&text=" + urlEncode(testo);
    if (conTastiera) body += "&reply_markup=" + urlEncode(theftMode ? tastieraFurto() : tastiera());
    long msgId = 0;
    for (int t = 0; t < 2 && msgId == 0; t++) {
        modem.https_begin();
        if (modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO)) {
            modem.https_set_content_type("application/x-www-form-urlencoded");
            int code = modem.https_post(body);
            if (code == 200) {
                String resp = modem.https_body();
                int p = resp.indexOf("\"message_id\":");
                if (p >= 0) { p += 13; msgId = resp.substring(p, resp.indexOf(",", p)).toInt(); }
                if (msgId == 0) msgId = -1;
            }
            Serial.printf("tgSend code=%d id=%ld kb=%d\n", code, msgId, conTastiera);
        }
        modem.https_end();
        if (msgId == 0) delay(800);
    }
    if (msgId == -1) msgId = 0;
    return msgId;
}

void tgDelete(long msgId) {
    if (msgId == 0) return;
    String url = "https://api.telegram.org/bot" TELEGRAM_TOKEN "/deleteMessage?chat_id=" TELEGRAM_CHAT_ID
                 "&message_id=" + String(msgId);
    modem.https_begin();
    if (modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO)) modem.https_get();
    modem.https_end();
}

void tgEdit(long msgId, String testo) {
    if (msgId == 0) return;
    String url = "https://api.telegram.org/bot" TELEGRAM_TOKEN "/editMessageText";
    String body = "chat_id=" TELEGRAM_CHAT_ID "&message_id=" + String(msgId)
                  + "&parse_mode=HTML&text=" + urlEncode(testo);
    modem.https_begin();
    if (modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO)) {
        modem.https_set_content_type("application/x-www-form-urlencoded");
        modem.https_post(body);
    }
    modem.https_end();
}

// LTE = accuracy>0 -> Telegram disegna il CERCHIO (raggio). GPS = accuracy 0 -> punto.
// Via POST (corpo) + log: cosi' si vede se Telegram accetta horizontal_accuracy.
long tgSendLocation(float lat, float lon, int livePeriod, int accuracy = 0) {
    String url = "https://api.telegram.org/bot" TELEGRAM_TOKEN "/sendLocation";
    String body = "chat_id=" TELEGRAM_CHAT_ID;
    body += "&latitude=" + String(lat, 6) + "&longitude=" + String(lon, 6);
    if (livePeriod > 0) body += "&live_period=" + String(livePeriod);
    int a = 0;
    if (accuracy > 0) { a = min(accuracy, 1500); body += "&horizontal_accuracy=" + String(a) + ".0"; }
    long id = 0;
    modem.https_begin();
    if (modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO)) {
        modem.https_set_content_type("application/x-www-form-urlencoded");
        int code = modem.https_post(body);
        String resp = modem.https_body();
        int p = resp.indexOf("\"message_id\":");
        if (p >= 0) { p += 13; id = resp.substring(p, resp.indexOf(",", p)).toInt(); }
        Serial.printf("tgSendLocation acc=%d live=%d code=%d -> %s\n", a, livePeriod, code, resp.substring(0, 90).c_str());
    }
    modem.https_end();
    return id;
}

void tgMoveLocation(long msgId, float lat, float lon, int accuracy = 0) {
    if (msgId == 0) return;
    String url = "https://api.telegram.org/bot" TELEGRAM_TOKEN "/editMessageLiveLocation?chat_id=" TELEGRAM_CHAT_ID;
    url += "&message_id=" + String(msgId);
    url += "&latitude=" + String(lat, 6) + "&longitude=" + String(lon, 6);
    if (accuracy > 0) { int a = min(accuracy, 1500); url += "&horizontal_accuracy=" + String(a); }
    modem.https_begin();
    if (modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO)) modem.https_get();
    modem.https_end();
}

// Ferma la condivisione live: il pin smette di "pulsare" e resta fisso sull'ultima
// posizione. Da chiamare quando si spegne il GPS (disarmo / fine sessione).
void tgStopLive(long msgId) {
    if (msgId == 0) return;
    String url = "https://api.telegram.org/bot" TELEGRAM_TOKEN "/stopMessageLiveLocation";
    String body = "chat_id=" TELEGRAM_CHAT_ID "&message_id=" + String(msgId);
    modem.https_begin();
    if (modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO)) {
        modem.https_set_content_type("application/x-www-form-urlencoded");
        modem.https_post(body);
    }
    modem.https_end();
}

// Mappa Geoapify come foto Telegram (bonus: se fallisce non e' critico)
long tgSendMapPhoto(float lat, float lon, int radiusMeters, String caption, bool precise = false) {
    int zoom = precise ? 16 : (radiusMeters > 800 ? 12 : 14);
    char mapUrl[280];
    snprintf(mapUrl, sizeof(mapUrl),
        "https://maps.geoapify.com/v1/staticmap?style=osm-bright&width=600&height=400"
        "&center=lonlat:%.4f,%.4f&zoom=%d&marker=lonlat:%.4f,%.4f&apiKey=" GEOAPIFY_KEY,
        lon, lat, zoom, lon, lat);

    String url = "https://api.telegram.org/bot" TELEGRAM_TOKEN "/sendPhoto?chat_id=" TELEGRAM_CHAT_ID;
    url += "&photo=" + urlEncode(String(mapUrl));
    if (caption.length() > 0) url += "&caption=" + urlEncode(caption) + "&parse_mode=HTML";

    Serial.printf("MapPhoto URL len=%d\n", url.length());
    long msgId = 0;
    modem.https_begin();
    if (modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO)) {
        int code = modem.https_get();
        String resp = modem.https_body();
        Serial.printf("sendPhoto: %d -> %s\n", code, resp.substring(0, 80).c_str());
        if (code == 200) {
            int p = resp.indexOf("\"message_id\":");
            if (p >= 0) { p += 13; msgId = resp.substring(p, resp.indexOf(",", p)).toInt(); }
        }
    }
    modem.https_end();
    return msgId;
}

// ne resta sempre una sola (cancella la precedente). Porta SEMPRE la tastiera,
// cosi' i bottoni restano disponibili dopo ogni cambio di stato.
void tgStatus(String testo) {
    long old = lastStatusMsgId;
    long neu = tgSend(testo, true);
    if (neu != 0) { lastStatusMsgId = neu; if (old != 0) tgDelete(old); }
}

void tgKeyboard() {
    tgSend(theftMode ? "\xF0\x9F\x9A\xA8 <b>Comandi furto</b>" : "\xF0\x9F\x8F\x8D <b>Comandi</b>", true);
}

// ===================== TELEGRAM RICEVI COMANDI =====================
void eseguiComando(String testo);   // forward declaration

void tgCheckCommands() {
    String url = "https://api.telegram.org/bot" TELEGRAM_TOKEN "/getUpdates?timeout=0&offset=";
    url += String(rtcLastUpdId + 1);
    modem.https_begin();
    String body = "";
    if (modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO)) {
        modem.https_get();
        body = modem.https_body();
    }
    modem.https_end();
    if (body.length() < 10) return;

    int idx = 0;
    while ((idx = body.indexOf("\"update_id\":", idx)) >= 0) {
        idx += 12;
        long id = body.substring(idx, body.indexOf(",", idx)).toInt();
        if (id > rtcLastUpdId) rtcLastUpdId = id;
    }
    if (body.indexOf("\"id\":" TELEGRAM_CHAT_ID) < 0) return;
    // Processa TUTTI i messaggi di testo pendenti (non solo l'ultimo): risponde a tutto.
    int p = 0;
    while ((p = body.indexOf("\"text\":\"", p)) >= 0) {
        p += 8;
        String testo = body.substring(p, body.indexOf("\"", p));
        if (testo.length() > 0) eseguiComando(testo);
    }
}

// Salta tutti i messaggi vecchi: imposta rtcLastUpdId all'ultimo esistente.
// Chiama una volta al primo boot per evitare replay di vecchi comandi (es. "calibra").
void skipOldMessages() {
    String url = "https://api.telegram.org/bot" TELEGRAM_TOKEN "/getUpdates?timeout=0&limit=1&offset=-1";
    modem.https_begin();
    String body = "";
    if (modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO)) {
        modem.https_get();
        body = modem.https_body();
    }
    modem.https_end();
    int idx = body.indexOf("\"update_id\":");
    if (idx >= 0) {
        idx += 12;
        long id = body.substring(idx, body.indexOf(",", idx)).toInt();
        if (id > rtcLastUpdId) {
            rtcLastUpdId = id;
            Serial.printf("skipOldMessages: lastUpdId=%ld\n", rtcLastUpdId);
        }
    }
}

// ===================== OROLOGIO DI RETE / SVEGLIA =====================
// Legge l'ora di rete UNA volta (a freddo, dopo il boot) e poi la calcola con millis(),
// senza ulteriori AT durante l'HTTPS. Cosi' i timestamp sono dell'EVENTO, non dell'invio.
void readClock() {
    String r = sendAT("AT+CCLK?", 500);
    int q = r.indexOf('"');
    if (q < 0) return;
    String t = r.substring(q + 1);          // "yy/MM/dd,HH:MM:SS+zz"  (zz = quarti d'ora)
    int comma = t.indexOf(',');
    if (comma < 0 || (int)t.length() < comma + 9) return;
    int hh = t.substring(comma + 1, comma + 3).toInt();
    int mm = t.substring(comma + 4, comma + 6).toInt();
    int ss = t.substring(comma + 7, comma + 9).toInt();
    // Fuso orario in QUARTI D'ORA dopo i secondi (es. "+08" = +2h). Il modem da' UTC,
    // quindi lo aggiungo per ottenere l'ora locale (Italia: +08 estate, +04 inverno).
    int tz = 0;
    if ((int)t.length() >= comma + 12) {
        int sign = (t.charAt(comma + 9) == '-') ? -1 : 1;
        tz = sign * t.substring(comma + 10, comma + 12).toInt();
    }
    long sec = (long)hh * 3600 + mm * 60 + ss + (long)tz * 15 * 60;
    bootClockSec = (sec % 86400 + 86400) % 86400;   // normalizza in [0,86400)
    bootClockMs  = millis();
}

String oraCorrente() {                       // "HH:MM:SS" ora locale, "--:--:--" se ignota
    if (bootClockSec < 0) return "--:--:--";
    long s = (bootClockSec + (long)((millis() - bootClockMs) / 1000)) % 86400;
    char buf[12];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", (int)(s / 3600), (int)((s % 3600) / 60), (int)(s % 60));
    return String(buf);
}

// Tieni il dispositivo sveglio (in loop) almeno per "ms" da adesso. Chiamata dai comandi.
void stayAwake(unsigned long ms) {
    unsigned long u = millis() + ms;
    if (u > awakeUntilMs) awakeUntilMs = u;
}

// ===================== SEGNALE E BATTERIA =====================
String barre(int n) {
    String s = "";
    for (int i = 0; i < 5; i++) s += (i < n) ? "\xE2\x96\xB0" : "\xE2\x96\xB1";
    return s;
}

String segnaleRecap() {
    int csq = modem.getSignalQuality();
    int lteLvl = (csq == 99 || csq == 0) ? 0 : (csq >= 20 ? 5 : csq >= 15 ? 4 : csq >= 10 ? 3 : csq >= 5 ? 2 : 1);
    int gpsLvl = (lastSat <= 0) ? 0 : (lastSat >= 8 ? 5 : lastSat >= 6 ? 4 : lastSat >= 4 ? 3 : lastSat >= 3 ? 2 : 1);
    String s = "\xF0\x9F\x93\xB6 LTE:  " + barre(lteLvl) + "  (" + String(csq) + "/31)\n";
    s += "\xF0\x9F\x9B\xB0 GPS: " + barre(gpsLvl);
    s += (gpsLvl == 0) ? "  (nessun fix)\n" : ("  (" + String(lastSat) + " sat)\n");
    return s;
}

String tempoOre(float h) {
    if (h < 0) h = 0;
    int ore = (int)h, min = (int)((h - ore) * 60);
    return (ore > 0) ? (String(ore) + "h " + String(min) + "m") : (String(min) + "m");
}

// "adesso" / "Xs fa" / "Xm Ys fa" dato un intervallo in millis (freschezza posizione)
String daQuando(unsigned long ms) {
    unsigned long s = ms / 1000;
    if (s < 5)  return "adesso";
    if (s < 60) return String(s) + "s fa";
    return String(s / 60) + "m " + String(s % 60) + "s fa";
}

// *** Batteria via AT+CBC. ***  GPIO4 e' risultato MORTO (0.001V) -> rimosso.
// AT+CBC risente del carico del modem: piu' il sistema e' attivo, piu' la tensione
// "sag". Per questo la misura buona si fa A MODEM FERMO, appena prima del deep sleep
// (vedi enterSleep): viene salvata in rtcBattV e riportata al messaggio successivo.
float cbcVolt() {   // una lettura AT+CBC -> V
    String r = sendAT("AT+CBC", 400);
    int p = r.indexOf("+CBC:");
    if (p < 0) return 0;
    int lastComma = r.lastIndexOf(',');
    int start = (lastComma > p) ? lastComma + 1 : p + 5;
    String num = "";
    for (int i = start; i < (int)r.length(); i++) {
        char c = r.charAt(i);
        if (isdigit(c) || c == '.') num += c;
        else if (num.length() > 0) break;
    }
    float v = num.toFloat();
    if (v > 100) v /= 1000.0;
    return v;
}

// Misura "quieta": MAX di piu' letture (filtra i cali momentanei) + offset di taratura.
void readBatteryQuiet() {
    float vmax = 0;
    for (int k = 0; k < 6; k++) { float v = cbcVolt(); if (v > vmax) vmax = v; delay(100); }
    float vcal = vmax + BATT_CAL_OFFSET;
    Serial.printf("Batteria (quieta): raw=%.3fV  corretta=%.3fV\n", vmax, vcal);
    if (vmax > 2.5 && vcal < 5.2) rtcBattV = vcal;
}

float batteryVolt() { return rtcBattV; }

// In carica: solo via soglia di tensione (AT+CBC da' la sola tensione su questa
// scheda). >=4.10V = in carica/CV. NB: a meta' carica (es. 3.8V) non e' rilevabile.
bool inCarica(float vbat) { return vbat >= 4.10; }

// % da tensione con curva LiPo approssimata (piu' realistica della lineare 3.3-4.2)
int pctFromV(float v) {
    const float vt[] = {3.30,3.50,3.60,3.70,3.75,3.80,3.85,3.90,3.95,4.00,4.10,4.20};
    const int   pt[] = {0,   9,   15,  28,  37,  47,  56,  66,  74,  82,  92,  100};
    if (v <= vt[0]) return 0;
    if (v >= vt[11]) return 100;
    for (int i = 0; i < 11; i++)
        if (v < vt[i+1]) return pt[i] + (int)((v - vt[i]) / (vt[i+1] - vt[i]) * (pt[i+1] - pt[i]));
    return 100;
}

// Autonomia "Xgg Yh" o "Xh Ym" dato consumo medio in mA
String tempoAutonomia(int pct, int mA) {
    float h = (BATTERY_MAH * (pct / 100.0)) / (float)mA;
    if (h >= 24) return String((int)(h / 24)) + "gg " + String((int)h % 24) + "h";
    return tempoOre(h);
}

// In furto mostra DUE autonomie (furto attivo + deep sleep) per un quadro completo.
String batteria(bool dormiMode = false) {
    float v = batteryVolt();
    int pct = pctFromV(v);
    String s = "\xF0\x9F\x94\x8B Batteria: <b>" + String(pct) + "%</b> (" + String(v, 2) + "V)\n";
    if (inCarica(v)) {
        float h = (BATTERY_MAH * (1.0 - pct / 100.0)) / CHARGE_MA;
        s += "\xE2\x9A\xA1 In carica - ~" + tempoOre(h) + " al 100%";
    } else if (theftMode) {
        int mAfurto = (trackMode == 1) ? 160 : 115;   // GPS costante consuma di piu'
        s += "\xF0\x9F\x9A\xA8 Furto: ~" + tempoAutonomia(pct, mAfurto) + " (~" + String(mAfurto) + "mA)\n";
        s += "\xF0\x9F\x92\xA4 Deep sleep: ~" + tempoAutonomia(pct, 1) + " (~1mA)";
    } else if (dormiMode) {
        s += "\xF0\x9F\x92\xA4 Sleep: ~" + tempoAutonomia(pct, 1) + " (~1mA)";
    } else {
        int mA = armed ? 70 : 55;
        s += "\xE2\x8F\xB3 Autonomia: ~" + tempoAutonomia(pct, mA) + " (~" + String(mA) + "mA)";
    }
    return s;
}

// ===================== GPS =====================
void gpsOn() {
    if (gpsActive) return;
    if (modem.enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL)) {
        modem.enableAGPS();
        gpsActive = true;
        Serial.println("GPS ON");
    }
}

void gpsOff() {
    if (!gpsActive) return;
    modem.disableGPS(MODEM_GPS_ENABLE_GPIO, !MODEM_GPS_ENABLE_LEVEL);
    gpsActive = false;
    Serial.println("GPS OFF");
}

bool getRoughLocation(String &out) {
    float lat = 0, lon = 0, acc = 0;
    int yr = 0, mo = 0, dy = 0, hr = 0, mi = 0, se = 0;
    if (modem.getGsmLocation(&lat, &lon, &acc, &yr, &mo, &dy, &hr, &mi, &se) && lat != 0) {
        lastLat = lat; lastLon = lon;
        lastAcc = (acc > 100) ? (int)acc : 1400;
        out = "\xF0\x9F\x93\xA1 <b>Zona approssimativa</b> (LTE, ~" + String(lastAcc) + "m)";
        return true;
    }
    return false;
}

bool getFix(String &out) {
    gpsOn();
    float lat = 0, lon = 0, spd = 0, alt = 0, acc = 0;
    int vsat = 0, usat = 0, yr = 0, mo = 0, dy = 0, hr = 0, mi = 0, se = 0;
    uint8_t fix = 0;
    for (int i = 0; i < 3; i++) {
        if (modem.getGPS(&fix, &lat, &lon, &spd, &alt, &vsat, &usat, &acc,
                         &yr, &mo, &dy, &hr, &mi, &se) && lat != 0) {
            lastLat = lat; lastLon = lon; lastSat = vsat; lastAcc = 0;
            String vel = (spd < 3.0) ? "fermo" : (String(spd, 1) + " km/h");
            out = "\xF0\x9F\x93\x8D <b>Posizione precisa</b> (GPS) \xC2\xB7 " + vel;
            return true;
        }
        delay(2000);
    }
    return false;
}

// ===================== REGISTRO VEICOLI (NVS, sopravvive allo spegnimento) =====================
#define MAX_VEICOLI 8

int vehCount()  { prefs.begin("veh", true); int n = prefs.getInt("count", 0);  prefs.end(); return n; }

String vehName(int i) {
    prefs.begin("veh", true);
    String n = prefs.getString(("n" + String(i)).c_str(), "");
    prefs.end();
    return n;
}

int vehFind(String name) {
    name.trim();
    int n = vehCount();
    for (int i = 0; i < n; i++) if (vehName(i).equalsIgnoreCase(name)) return i;
    return -1;
}

// Carica le soglie del veicolo i in rtcSens* e lo rende attivo
void vehApply(int i) {
    prefs.begin("veh", true);
    rtcSensTocco  = prefs.getLong(("t" + String(i)).c_str(), DEFAULT_SENS_TOCCO);
    rtcSensSeduto = prefs.getLong(("s" + String(i)).c_str(), DEFAULT_SENS_SEDUTO);
    rtcSensMuove  = prefs.getLong(("m" + String(i)).c_str(), DEFAULT_SENS_MUOVE);
    prefs.end();
    prefs.begin("veh", false); prefs.putInt("active", i); prefs.end();
    rtcSensMult = 100; threshold = effTocco();
}

// All'avvio a freddo ricarica il veicolo attivo (la calibrazione sopravvive allo spegnimento)
void vehApplyActive() {
    prefs.begin("veh", true);
    int a = prefs.getInt("active", -1), n = prefs.getInt("count", 0);
    prefs.end();
    if (a >= 0 && a < n) vehApply(a);
}

// Salva le soglie correnti (rtcSens*) sul veicolo "name" (sovrascrive o crea). -1 = memoria piena
int vehSave(String name) {
    name.replace("\"", ""); name.replace("\\", ""); name.trim();
    if (name.length() == 0)  name = "Veicolo";
    if (name.length() > 24)  name = name.substring(0, 24);
    int i = vehFind(name), n = vehCount();
    if (i < 0) { if (n >= MAX_VEICOLI) return -1; i = n; }
    prefs.begin("veh", false);
    prefs.putString(("n" + String(i)).c_str(), name);
    prefs.putLong(("t" + String(i)).c_str(), rtcSensTocco);
    prefs.putLong(("s" + String(i)).c_str(), rtcSensSeduto);
    prefs.putLong(("m" + String(i)).c_str(), rtcSensMuove);
    if (i >= n) prefs.putInt("count", i + 1);
    prefs.putInt("active", i);
    prefs.end();
    return i;
}

// Tastiera Telegram coi nomi dei veicoli salvati (+ "Nuovo veicolo" se conNuovo)
String vehKeyboard(bool conNuovo) {
    String kb = "{\"keyboard\":[";
    int n = vehCount();
    for (int i = 0; i < n; i++) kb += "[\"" + vehName(i) + "\"],";
    if (conNuovo) kb += "[\"\xE2\x9E\x95 Nuovo veicolo\"],";
    if (kb.endsWith(",")) kb.remove(kb.length() - 1);
    kb += "],\"resize_keyboard\":true,\"one_time_keyboard\":true}";
    return kb;
}

// Invia un messaggio con una tastiera personalizzata (POST, come tgSend)
long tgSendCustomKb(String testo, String kbJson) {
    String url = "https://api.telegram.org/bot" TELEGRAM_TOKEN "/sendMessage";
    String body = "chat_id=" TELEGRAM_CHAT_ID "&parse_mode=HTML&text=" + urlEncode(testo)
                  + "&reply_markup=" + urlEncode(kbJson);
    long id = 0;
    modem.https_begin();
    if (modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO)) {
        modem.https_set_content_type("application/x-www-form-urlencoded");
        if (modem.https_post(body) == 200) {
            String resp = modem.https_body();
            int p = resp.indexOf("\"message_id\":");
            if (p >= 0) { p += 13; id = resp.substring(p, resp.indexOf(",", p)).toInt(); }
        }
    }
    modem.https_end();
    return id;
}

String elencoVeicoli() {
    int n = vehCount();
    if (n == 0) return "<i>Nessun veicolo salvato.</i>";
    String s = "";
    for (int i = 0; i < n; i++) s += "\xF0\x9F\x9A\x97 " + vehName(i) + "\n";
    return s;
}

// ===================== CLASSIFICAZIONE MOVIMENTO =====================
// Misura peak magnitude per 1.5s, ne ricava il livello (1=tocca 2=seduto 3=muove)
// e lo salva in lastEventLevel (per la ri-chiamata su escalation).
int lastEventLevel = 0;   // livello dell'ultimo evento classificato

String classificaMovimento() {
    mpu.getIntStatus();  // pulisci interrupt MPU
    long peakMag = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 1500) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        long m = abs(gx) + abs(gy) + abs(gz);
        if (m > peakMag) peakMag = m;
        delay(40);
    }
    lastEventLevel = livelloMovimento(peakMag);
    if (lastEventLevel == 0) lastEventLevel = 1;   // ha svegliato la MPU: minimo "tocca"
    Serial.printf("Peak magnitude: %ld -> livello %d (tocca=%ld seduto=%ld muove=%ld, mult=%d%%)\n",
                  peakMag, lastEventLevel, effTocco(), effSeduto(), effMuove(), rtcSensMult);
    return testoLivello(lastEventLevel);
}

// ===================== STATO =====================
String statoSistema() {
    String s = "<b>\xF0\x9F\x8F\x8D Allarme Moto</b>\n\n";
    s += armed ? "Stato: \xF0\x9F\x94\xB4\xF0\x9F\x94\x92 ATTIVO\n" : "Stato: \xF0\x9F\x9F\xA2\xF0\x9F\x94\x93 DISATTIVO\n";
    s += "Chiamate: " + String(mutedCalls ? "\xF0\x9F\x94\x95 sospese" : "\xF0\x9F\x94\x94 attive") + "\n";
    String sensStr = (rtcSensMult == 130) ? "Alta +30% (anti-pioggia)" :
                     (rtcSensMult == 80)  ? "Bassa -20% (scatta prima)" : "Default (calibrata)";
    s += "Sensibilita: " + sensStr + " - scatto \xE2\x89\xa5" + String(effTocco()) + "\n";
    s += "Livelli: \xF0\x9F\x91\x8A tocca=" + String(effTocco()) +
         " \xF0\x9F\x9A\xB2 seduto=" + String(effSeduto()) +
         " \xF0\x9F\x9A\x97 muove=" + String(effMuove()) + "\n";
    s += "Sleep: attivo (wakeup su movimento + check 4h)\n\n";
    s += segnaleRecap();
    s += batteria(armed);  // sleep mode se armato, attivo se sveglio
    return s;
}

// ===================== CALIBRAZIONE 3 LIVELLI =====================
long samplePeak(int durationMs) {
    long peak = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < (unsigned long)durationMs) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        long m = abs(gx) + abs(gy) + abs(gz);
        if (m > peak) peak = m;
        delay(40);
    }
    return max(peak, 500L);
}

void calibrazione() {
    stayAwakeStart = millis();
    tgSend("\xF0\x9F\x8E\x9A <b>Calibrazione 3 livelli</b>\n\nTre fasi da 5 secondi ciascuna.\nPreparati...");
    delay(3000);

    // Fase 1: tocca (vibrazione lieve)
    tgSend("\xF0\x9F\x91\x8A <b>FASE 1/3 - TOCCA</b>\nTocca/sfiora la moto come un urto LIEVE.\n\xE2\x8F\xB1 Via in 3 secondi...");
    delay(3000);
    long val1 = samplePeak(5000);
    tgSend("\xE2\x9C\x85 Fase 1 registrata: " + String(val1));
    delay(1500);

    // Fase 2: qualcuno seduto (vibrazione media)
    tgSend("\xF0\x9F\x9A\xB2 <b>FASE 2/3 - SEDUTO</b>\nSimula qualcuno che si SIEDE sulla moto.\n\xE2\x8F\xB1 Via in 3 secondi...");
    delay(3000);
    long val2 = samplePeak(5000);
    tgSend("\xE2\x9C\x85 Fase 2 registrata: " + String(val2));
    delay(1500);

    // Fase 3: muove/sposta (vibrazione forte)
    tgSend("\xF0\x9F\x9A\x97 <b>FASE 3/3 - SPOSTA</b>\nSPOSTA/scuoti forte la moto (movimento deciso).\n\xE2\x8F\xB1 Via in 3 secondi...");
    delay(3000);
    long val3 = samplePeak(5000);
    tgSend("\xE2\x9C\x85 Fase 3 registrata: " + String(val3));

    // Ordina i 3 valori in crescente: min=tocca, mid=seduto, max=muove
    long vals[3] = {val1, val2, val3};
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2 - i; j++)
            if (vals[j] > vals[j+1]) { long t = vals[j]; vals[j] = vals[j+1]; vals[j+1] = t; }

    rtcSensTocco  = vals[0];   // piu' basso = tocca
    rtcSensSeduto = vals[1];
    rtcSensMuove  = vals[2];   // piu' alto = muove/sposta
    rtcSensMult   = 100;       // torna a sensibilita' Default
    threshold     = effTocco(); // scatta al livello piu' lieve (tocca)
    alerted       = false;

    String msg = "\xE2\x9C\x85 <b>Calibrazione completata!</b>\n\n";
    msg += "\xF0\x9F\x91\x8A Tocca (lieve): \xE2\x89\xa5" + String(rtcSensTocco) + "\n";
    msg += "\xF0\x9F\x9A\xB2 Seduto (media): \xE2\x89\xa5" + String(rtcSensSeduto) + "\n";
    msg += "\xF0\x9F\x9A\x97 Sposta (forte): \xE2\x89\xa5" + String(rtcSensMuove) + "\n\n";
    msg += "\xF0\x9F\x92\xBE <b>Su quale veicolo salvo?</b>\nScegli un veicolo esistente o crea un nuovo veicolo.";
    tgSendCustomKb(msg, vehKeyboard(true));
    awaitVehSave   = true;          // la prossima risposta sceglie il veicolo
    stayAwakeStart = millis();      // reset timer dopo calibrazione
}

// ===================== NTFY (Shortcut iPhone) =====================
void ntfyCheck() {
    String url = "https://ntfy.sh/" NTFY_TOPIC "/json?poll=1&since=300s";
    modem.https_begin();
    String body = "";
    if (modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO)) {
        modem.https_get();
        body = modem.https_body();
    }
    modem.https_end();
    if (body.length() < 5) return;

    int pid = body.lastIndexOf("\"id\":\"");
    if (pid >= 0) {
        pid += 6;
        String id = body.substring(pid, body.indexOf("\"", pid));
        if (id == String(rtcNtfyId)) return;
        strncpy(rtcNtfyId, id.c_str(), 39);
    }
    int p = body.lastIndexOf("\"message\":\"");
    if (p < 0) return;
    p += 11;
    String msg = "";
    for (int i = p; i < (int)body.length(); i++) {
        char c = body.charAt(i);
        if (c == '\\' && i + 1 < (int)body.length()) { msg += body.charAt(++i); continue; }
        if (c == '"') break;
        msg += c;
    }
    if (msg.length() > 0) eseguiComando(msg);
}

// ===================== TWILIO =====================
void chiamaTwilio() {
    Serial.println("Twilio...");
    String url = "https://api.twilio.com/2010-04-01/Accounts/" TWILIO_SID "/Calls.json";
    String body = "To=" + urlEncode(CALL_TO) + "&From=" + urlEncode(TWILIO_FROM)
                  + "&Url=" + urlEncode("http://demo.twilio.com/docs/voice.xml");
    modem.https_begin();
    modem.https_set_url(url.c_str(), TINYGSM_SSL_AUTO);
    modem.https_set_content_type("application/x-www-form-urlencoded");
    modem.https_add_header("Authorization", TWILIO_AUTH);
    int code = modem.https_post(body);
    Serial.print("Twilio: "); Serial.println(code);
    modem.https_end();
}

// ===================== TRACKING (Modalita Furto / Posizione live) =====================
void trackingTick() {
    if (!theftMode && !liveTracking) return;   // difensivo: non girare mai spuriamente
    unsigned long now = millis();
    int pct = pctFromV(batteryVolt());
    unsigned long interval;
    if (trackMode == 1)     interval = 5000;
    else if (pct > 50)      interval = 10000;
    else if (pct > 20)      interval = 20000;
    else if (pct > 10)      interval = 40000;
    else                    interval = 90000;

    if (now - lastTrack < interval) return;
    lastTrack = now;
    readBatteryQuiet();   // modem idle tra un tick e l'altro: lettura batteria pulita

    // Prova GPS (in GPS-costante o se batteria >10%); altrimenti ripiega su LTE
    String posText; bool gotGps = false, gotLte = false;
    if (trackMode == 1 || pct > 10) {
        String g; if (getFix(g)) { gotGps = true; posText = g; }
    }
    if (!gotGps && getRoughLocation(posText)) gotLte = true;

    // *** UNA sola live location ***: accuracy = raggio (LTE -> CERCHIO stile Dov'e')
    // oppure 0 (GPS -> punto preciso). La live mostra il cerchio, la statica no.
    if (gotGps || gotLte) {
        int acc = gotGps ? 0 : min(lastAcc, 1500);
        if (gotGps) { lastGoodPos = posText; lastGpsFixMs = now; }
        if (trackLocId == 0) trackLocId = tgSendLocation(lastLat, lastLon, 3600, acc);
        else                 tgMoveLocation(trackLocId, lastLat, lastLon, acc);
        // Caption SOTTO la mappa (sorgente + ora). Sostituita al passaggio LTE->GPS.
        String cap = gotGps
            ? ("\xF0\x9F\x9B\xB0 <b>GPS preciso</b> - ultimo aggiornamento ore " + oraCorrente())
            : ("\xF0\x9F\x93\xA1 <b>Zona approssimativa LTE</b> (~" + String(lastAcc) + "m, cerchio) - GPS in arrivo...");
        if (trackCapId == 0) trackCapId = tgSend(cap, false);
        else                 tgEdit(trackCapId, cap);
    }

    // Messaggio di STATO separato: SOLO modo, segnale, batteria (niente info sorgente).
    String head = (liveTracking && !theftMode) ? "\xF0\x9F\x93\x8D <b>POSIZIONE LIVE</b>\n"
                                               : "\xF0\x9F\x9A\xA8 <b>MODALITA FURTO - TRACKING</b>\n";
    head += "Modo: " + String(trackMode == 1 ? "\xF0\x9F\x9B\xB0 GPS costante (5s)" : "\xE2\x9A\xA1 Risparmio") + "\n";
    head += "Aggiorna ogni " + String((int)(interval / 1000)) + "s\n\n";
    if (!gotGps && !gotLte) head += "\xE2\x9A\xA0 Posizione non disponibile, ricerca...\n\n";
    head += segnaleRecap() + batteria();
    if (trackMsgId == 0) trackMsgId = tgSend(head);
    else                 tgEdit(trackMsgId, head);
}

// ===================== COMANDI =====================
void eseguiComando(String testo) {
    String raw = testo; raw.trim();   // nome veicolo: serve la versione con maiuscole
    testo.toLowerCase();
    Serial.print("Cmd: "); Serial.println(testo);

    // *** Anti-eco ***: ignora i messaggi MULTI-RIGA. I bottoni e le Shortcut sono
    // sempre su una riga; i messaggi di STATO del bot (rispediti indietro da una
    // Shortcut mal configurata) contengono "\n" -> non sono comandi. Senza questo,
    // "Antifurto online" attivava la modalita' furto, "Allarme attivato" ri-armava, ecc.
    if (testo.indexOf("\\n") >= 0 || testo.indexOf('\n') >= 0) {
        Serial.println("(ignorato: messaggio multi-riga / eco di stato)");
        return;
    }

    // Ogni comando ricevuto = stai interagendo -> tieni sveglio 1 minuto (cosi' i flussi
    // multi-step come Calibra->salva-veicolo non si addormentano a meta').
    stayAwake(60000);

    // --- REGISTRO VEICOLI: stati di attesa (gestiti PRIMA dei comandi normali) ---
    if (awaitVehName) {                       // attendo il nome del nuovo veicolo
        awaitVehName = false;
        int i = vehSave(raw);
        tgKeyboard();
        if (i >= 0) tgStatus("\xF0\x9F\x92\xBE Veicolo <b>" + vehName(i) + "</b> creato e selezionato.\n\n" + statoSistema());
        else        tgSend("\xE2\x9A\xA0 Memoria piena (max " + String(MAX_VEICOLI) + " veicoli).", true);
        return;
    }
    if (awaitVehSave) {                        // attendo: su quale veicolo salvo la calibrazione
        awaitVehSave = false;
        if (raw.indexOf("uovo veicolo") >= 0 || testo.indexOf("nuovo") >= 0) {
            awaitVehName = true; stayAwakeStart = millis();
            tgSend("\xF0\x9F\x93\x9D Scrivi il <b>nome</b> del nuovo veicolo:", false);
        } else {
            int i = vehSave(raw);
            tgKeyboard();
            tgStatus("\xF0\x9F\x92\xBE Calibrazione salvata su <b>" + (i >= 0 ? vehName(i) : raw) + "</b>.\n\n" + statoSistema());
        }
        return;
    }
    if (awaitVehSelect) {                       // attendo: quale veicolo caricare
        int i = vehFind(raw);
        if (i >= 0) {
            awaitVehSelect = false;
            vehApply(i);
            tgKeyboard();
            tgStatus("\xF0\x9F\x9A\x97 Veicolo <b>" + vehName(i) + "</b> caricato.\n\n" + statoSistema());
            return;
        }
        // se non e' un nome valido, annullo l'attesa e lascio gestire il comando normale
        awaitVehSelect = false;
        tgKeyboard();
    }

    if (testo.indexOf("esci") >= 0) {
        if (trackLocId) tgStopLive(trackLocId);   // ferma la live (GPS si spegne)
        theftMode = false; liveTracking = false; trackMsgId = 0; trackLocId = 0; trackCapId = 0;
        gpsOff();
        tgSend("\xE2\x9C\x96 <b>Furto disattivato</b>", true);
    }
    else if (testo.indexOf("live") < 0 && (testo.indexOf("gps costante") >= 0 || testo.indexOf("costante") >= 0)) {
        trackMode = 1; lastTrack = 0;
        tgStatus("\xF0\x9F\x9B\xB0 <b>GPS costante</b> - ogni 5s");
    }
    else if (testo.indexOf("live") < 0 && testo.indexOf("risparmio") >= 0) {
        trackMode = 0; lastTrack = 0;
        tgStatus("\xE2\x9A\xA1 <b>Risparmio</b> - frequenza da batteria");
    }
    else if (testo.indexOf("furto") >= 0 && testo.indexOf("antifurto") < 0) {  // non "antifurto"
        theftMode = true; armed = true; alerted = false; forceAwake = true;
        trackMode = 0; trackMsgId = 0; trackLocId = 0; trackCapId = 0; lastTrack = 0;
        rtcCooldownMsgSent = false;
        gpsOn();
        tgSend("\xF0\x9F\x9A\xA8 <b>MODALITA FURTO ATTIVA</b>\nScegli \xE2\x9A\xA1 Risparmio o \xF0\x9F\x9B\xB0 GPS costante.", true);
    }
    else if (testo.indexOf("veicol") >= 0) {   // "Seleziona veicolo"
        stayAwakeStart = millis();   // ~3 min per scegliere
        if (vehCount() == 0) {
            tgSend("\xF0\x9F\x9A\x97 <b>Nessun veicolo salvato.</b>\nFai prima \xF0\x9F\x8E\x9A Calibra e salvane uno.", true);
        } else {
            tgSendCustomKb("\xF0\x9F\x9A\x97 <b>Seleziona veicolo</b>\nTocca il veicolo da caricare:\n\n" + elencoVeicoli(),
                           vehKeyboard(false));
            awaitVehSelect = true;
        }
    }
    else if (testo.indexOf("stop chiam") >= 0 || testo.indexOf("silenzia") >= 0) {
        mutedCalls = true;
        tgStatus("\xF0\x9F\x94\x95 <b>Chiamate sospese per questo allarme</b>\n"
                 "<i>Utile durante un inseguimento sotto i tuoi occhi. Si riattivano "
                 "da sole a fine inseguimento (10 min senza movimenti) o con \xF0\x9F\x94\x94 Riattiva.</i>");
    }
    else if (testo.indexOf("riattiva") >= 0) {
        mutedCalls = false;
        tgStatus("\xF0\x9F\x94\x94 <b>Chiamate riattivate</b>");
    }
    else if (testo.indexOf("disattiva") >= 0) {
        bool eraFurto = theftMode;
        // Ferma le live location attive: col GPS spento non devono restare "vive"
        if (rtcLiveLocMsgId) tgStopLive(rtcLiveLocMsgId);
        if (rtcAlarmPinId)   tgStopLive(rtcAlarmPinId);   // zona LTE (ora e' live)
        if (trackLocId)      tgStopLive(trackLocId);
        if (rtcGpsCapId && lastGpsFixMs)
            tgEdit(rtcGpsCapId, "\xF0\x9F\x9B\xB0 <b>Ultima posizione GPS</b> (disattivato)\n"
                                "\xE2\x9A\xA0 NON aggiornata - ultimo fix: " + daQuando(millis() - lastGpsFixMs));
        armed = false; theftMode = false; trackMsgId = 0; trackLocId = 0; trackCapId = 0;
        rtcLastAlarmMsgId = 0;   // prossimo allarme: nuovo messaggio fresco
        rtcLiveLocMsgId   = 0;   // prossima live location: nuova
        rtcAlarmPinId = 0; rtcLteCapId = 0; rtcGpsCapId = 0;
        maxLevelCalled = 0; callsMade = 0;
        rtcDisarmPhase = 0; rtcCooldownMsgSent = false;
        gpsOff();
        justSwitched = true; stayAwakeStart = millis();  // 30s per ripensarci
        if (eraFurto) tgSend("\xF0\x9F\x8F\x8D Comandi", true);
        tgStatus("\xF0\x9F\x9F\xA2\xF0\x9F\x94\x93 <b>Allarme DISATTIVATO</b>\n\n" + segnaleRecap() + batteria(true));
    }
    else if (testo.indexOf("attiva") >= 0) {
        armed = true; alerted = false;
        rtcCooldownMsgSent = false;  // switch -> il prossimo movimento da disarmato richiedera' di nuovo
        rtcDisarmPhase = 0;
        justSwitched = true; stayAwakeStart = millis();  // 30s per ripensarci
        tgStatus("\xF0\x9F\x94\xB4\xF0\x9F\x94\x92 <b>Allarme ATTIVATO</b>\n\n" + segnaleRecap() + batteria(true));
    }
    else if (testo.indexOf("ferma live") >= 0 || testo.indexOf("stop live") >= 0) {
        if (trackLocId) tgStopLive(trackLocId);
        liveTracking = false; liveUntilMs = 0; trackMsgId = 0; trackLocId = 0; trackCapId = 0;
        gpsOff();
        tgStatus("\xE2\x9C\x96 <b>Posizione live fermata</b>");
    }
    else if (testo.indexOf("live") >= 0) {     // "Rendi live (risparmio/5s)": senza scadenza
        liveTracking = true;
        liveUntilMs = 0;   // permanente: niente auto-stop
        trackMode = (testo.indexOf("5s") >= 0 || testo.indexOf("costante") >= 0 || testo.indexOf("gps") >= 0) ? 1 : 0;
        trackMsgId = 0; trackLocId = 0; trackCapId = 0; lastTrack = 0;
        gpsOn();
        tgStatus("\xF0\x9F\x93\x8D <b>Posizione LIVE avviata</b> (" + String(trackMode == 1 ? "GPS 5s" : "risparmio")
                 + ")\nResta attiva finche' non premi \xE2\x9C\x96 Ferma live.");
    }
    else if (testo.indexOf("posizione") >= 0) {
        // Avvia un LIVE di 10 minuti: trackingTick manda subito la zona LTE (live col
        // CERCHIO) e poi la sostituisce col GPS preciso appena aggancia.
        liveTracking = true; trackMode = 0; lastTrack = 0;
        trackMsgId = 0; trackLocId = 0; trackCapId = 0;
        liveUntilMs = millis() + 600000;   // auto-stop dopo 10 minuti
        stayAwake(600000);
        gpsOn();
        tgSendCustomKb("\xF0\x9F\x93\x8D <b>Posizione live (10 min)</b>\n"
                       "Mando la zona LTE (cerchio) e cerco il GPS preciso; la mappa si aggiorna da sola.\n"
                       "Usa \xF0\x9F\x93\x8D Rendi live per non farla scadere, o \xE2\x9C\x96 Ferma live.",
                       "{\"keyboard\":[[\"\xF0\x9F\x93\x8D Rendi live risparmio\",\"\xF0\x9F\x9B\xB0 Rendi live 5s\"],"
                       "[\"\xE2\x9C\x96 Ferma live\",\"\xE2\x84\xB9 Stato\"]],\"resize_keyboard\":true}");
    }
    else if (testo.indexOf("stato") >= 0) {
        tgSend(statoSistema(), false);
    }
    else if (testo.indexOf("calibra") >= 0) {
        calibrazione();
    }
    else if (testo.indexOf("alta") >= 0) {
        sensMult = 130; threshold = effTocco();   // +30%: soglie piu' alte, anti-pioggia
        tgStatus("\xF0\x9F\x94\xBC Sensibilita <b>ALTA</b> (+30%, anti-pioggia)\nScatta a \xE2\x89\xa5" + String(effTocco()));
    }
    else if (testo.indexOf("default") >= 0) {
        sensMult = 100; threshold = effTocco();   // valori di calibra esatti
        tgStatus("\xE2\x9E\x96 Sensibilita <b>DEFAULT</b> (valori calibrati)\nScatta a \xE2\x89\xa5" + String(effTocco()));
    }
    else if (testo.indexOf("bassa") >= 0) {
        sensMult = 80; threshold = effTocco();    // -20%: soglie piu' basse, scatta prima
        tgStatus("\xF0\x9F\x94\xBD Sensibilita <b>BASSA</b> (-20%, scatta prima)\nScatta a \xE2\x89\xa5" + String(effTocco()));
    }
    else if (testo.indexOf("/start") >= 0 || testo.indexOf("menu") >= 0 || testo.indexOf("comandi") >= 0) {
        tgKeyboard();
        tgSend(statoSistema(), false);
    }
}

// ===================== AVVIO MODEM =====================
void powerOnModem() {
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
#ifdef MODEM_RESET_PIN
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL); delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);  delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
#endif
#ifdef MODEM_DTR_PIN
    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, LOW);
#endif
    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);  delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH); delay(MODEM_POWERON_PULSE_WIDTH_MS);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    if (!checkRespond()) { Serial.println("Aspetto avvio modem..."); delay(MODEM_START_WAIT_MS); }
    sendAT("ATE0");
}

bool waitRegistration(int maxSeconds) {
    int elapsed = 0;
    while (elapsed < maxSeconds) {
        String r = sendAT("AT+CEREG?");
        if (r.indexOf(",1") >= 0 || r.indexOf(",5") >= 0) return true;
        delay(3000); elapsed += 4;
    }
    return false;
}

bool activateData() {
    sendAT("AT+CGDCONT=1,\"IP\",\"" APN "\"", 1000);
    sendAT("AT+CGACT=1,1", 6000);
    String ip = sendAT("AT+CGPADDR=1", 1000);
    return (ip.indexOf(".") >= 0 && ip.indexOf("0.0.0.0") < 0);
}

bool bootModem(int maxSec = 300) {
    Serial.println("Boot modem...");
    powerOnModem();
    sendAT("AT+CPIN?");
    if (!waitRegistration(maxSec)) {
        Serial.println("Nessun segnale, torno in sleep");
        return false;
    }
    int tentativi = 0;
    while (!activateData() && tentativi++ < 3) delay(5000);
    if (tentativi > 3) { Serial.println("Dati falliti"); return false; }
    Serial.println("Connesso!");
    modemReady = true;
    readClock();                          // ora di rete per i timestamp (evento, non invio)
    if (rtcBattV == 0) readBatteryQuiet(); // primo boot assoluto: evita di mostrare 0%
    return true;
}

// ===================== DEEP SLEEP =====================
void configuraMpuInterrupt() {
    pinMode(MPU_INT_PIN, INPUT);      // esplicitamente INPUT (non held)
    Wire.begin(I2C_SDA, I2C_SCL);
    mpu.initialize();
    mpu.setSleepEnabled(false);

    // *** SEQUENZA CANONICA WAKE-ON-MOTION (la parte che mancava) ***
    // Il filtro passa-alto (DHPF) e' OBBLIGATORIO: senza, il motore di
    // motion-detection della MPU NON genera mai l'interrupt sul pin INT,
    // quindi GPIO6 resta LOW e EXT1 non sveglia mai l'ESP32.
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);
    mpu.setDHPFMode(MPU6050_DHPF_5);          // <-- critico per il wakeup
    mpu.setAccelerometerPowerOnDelay(3);
    mpu.setMotionDetectionThreshold(10);      // sensibilita' hardware
    mpu.setMotionDetectionDuration(2);        // ms sopra soglia

    // Pin INT: attivo ALTO, push-pull, latched (resta HIGH fino alla lettura),
    // e si azzera leggendo qualsiasi registro.
    mpu.setInterruptMode(0);                  // 0 = active high
    mpu.setInterruptDrive(0);                 // 0 = push-pull
    mpu.setInterruptLatch(1);                 // 1 = latched: INT sale e CI RESTA
    mpu.setInterruptLatchClear(1);            // azzera ad ogni lettura

    // Abilita SOLO l'interrupt di movimento (bit6 = MOT_EN)
    mpu.setIntEnabled(0x40);
}

// Calcola minuti fino al prossimo checkpoint orario (00,04,08,12,16,20)
int minutiAlProssimoCheckIn() {
    String r = sendAT("AT+CCLK?", 1000);
    int comma = r.indexOf(',');
    if (comma < 0) return 240;
    String t = r.substring(comma + 1);
    int hour = t.substring(0, 2).toInt();
    int mins = t.substring(3, 5).toInt();
    int nowMin = hour * 60 + mins;
    int checkpoints[] = {0, 4, 8, 12, 16, 20};
    for (int i = 0; i < 6; i++) {
        int cp = checkpoints[i] * 60;
        if (cp > nowMin) return cp - nowMin + 2;
    }
    return (24 * 60 - nowMin) + 2;
}

void enterSleep() {
    gpsOff();
    configuraMpuInterrupt();

    // *** CRITICO: pulisci interrupt MPU PRIMA di configurare EXT1 ***
    // Se GPIO6 è HIGH (interrupt pendente), gpio_hold lo fisserebbe HIGH
    // causando wakeup istantaneo. Leggendo getIntStatus(), GPIO6 torna LOW.
    mpu.getIntStatus();

    int minsToSleep;
    bool suppressMotion = false;

    if (!rtcArmed && rtcDisarmPhase == 1) {
        // Fase IGNORA: dormi 1 minuto ignorando completamente la MPU.
        // Il timer sveglia in handleCheckIn, che passa alla fase OSSERVA.
        minsToSleep    = 1;
        suppressMotion = true;
    } else if (!rtcArmed && rtcDisarmPhase == 2) {
        // Fase OSSERVA: dormi max 1 minuto con la MPU ATTIVA.
        //   timer -> minuto tranquillo, torno in sleep normale (handleCheckIn)
        //   moto  -> nuovo movimento, ricontrollo lo stato (handleMotionDisarmed)
        minsToSleep    = 1;
        suppressMotion = false;
    } else {
        minsToSleep     = minutiAlProssimoCheckIn();
        suppressMotion  = false;
        rtcDisarmPhase  = 0;
    }

    // Accumula il tempo dall'ultimo "Stai spostando?" (per il reinvio a 6h)
    rtcMinSinceMoveMsg += minsToSleep;

    // *** DIAGNOSTICO: GPIO6 deve essere LOW prima di dormire ***
    int gpio6state = digitalRead(MPU_INT_PIN);
    Serial.printf("Sleep %d min | cooldown=%d | suppress=%d | GPIO6=%d\n",
                  minsToSleep, rtcCooldownMsgSent, suppressMotion, gpio6state);
    if (gpio6state == HIGH) {
        Serial.println("WARN: GPIO6 ancora HIGH! Pulisco interrupt MPU...");
        mpu.getIntStatus();
        delay(10);
        Serial.printf("GPIO6 dopo pulizia: %d\n", digitalRead(MPU_INT_PIN));
    }

    // *** CRITICO: tieni MODEM_RESET_PIN HIGH durante sleep ***
    // Senza hold, il pin può fluttuare e far ripartire il modem (issue #85 LilyGo)
    // Vedi: DeepSleep.ino ufficiale LilyGo
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);  // HIGH (inattivo per SIM7670G)
    gpio_hold_en((gpio_num_t)MODEM_RESET_PIN);
    gpio_deep_sleep_hold_en();

    // *** NON fare gpio_hold su GPIO6 (MPU_INT_PIN) ***
    // GPIO6 è guidato direttamente dal MPU-6050. Hold su INPUT causa
    // conflitto con il driver esterno e impedisce il wakeup.

    if (!suppressMotion) {
        // Pulldown sul pin INT: idle pulito a LOW. Il push-pull della MPU vince
        // sul pulldown quando porta HIGH -> evita risvegli spuri da pin flottante.
        rtc_gpio_pulldown_en((gpio_num_t)MPU_INT_PIN);
        rtc_gpio_pullup_dis((gpio_num_t)MPU_INT_PIN);
        esp_sleep_enable_ext1_wakeup(1ULL << MPU_INT_PIN, ESP_EXT1_WAKEUP_ANY_HIGH);
    }
    esp_sleep_enable_timer_wakeup((uint64_t)minsToSleep * 60ULL * 1000000ULL);

    // *** Misura batteria a MODEM FERMO (niente HTTPS/GPS in corso) ***
    // Solo se il modem e' acceso (altrimenti AT+CBC torna vuoto e si perde tempo).
    if (modemReady) {
        delay(400);            // lascia assestare il rail
        readBatteryQuiet();
    }

    modem.poweroff();
    modemReady = false;
    delay(300);
    Serial.println("ZZZ...");
    Serial.flush();
    delay(100);
    esp_deep_sleep_start();
}

// ===================== ALLARME (logica unica) =====================
// Usata sia dal wakeup da deep sleep (handleAlarm) sia da loop() quando
// la moto si muove mentre siamo gia' svegli. Nessuna duplicazione.

// PRIORITA': far squillare il telefono ASAP, poi notificare. Ordine:
//   1) CHIAMATA Twilio (subito)
//   2) messaggio ALLARME con tastiera (evento + segnale + batteria, NIENTE info LTE qui)
//   3) pin LTE col RAGGIO + caption "zona LTE" sotto (sara' tutto sostituito dal GPS)
// Assume il modem gia' connesso.
void eseguiAllarme(String tipoEvento) {
    bool primoEvento = (rtcLastAlarmMsgId == 0);

    // 1) CHIAMATA SUBITO (chiamata #1; livello per cui ho chiamato = quello dell'evento)
    if (!mutedCalls) { chiamaTwilio(); lastCall = millis(); callsMade++; }
    maxLevelCalled = lastEventLevel;

    // 2) MESSAGGIO ALLARME con tastiera (nessuna info LTE qui dentro)
    String posText = (lastGoodPos.length() > 0) ? ("<i>Ultima GPS:</i>\n" + lastGoodPos + "\n\n") : "";
    String alarmBody = "\xF0\x9F\x9A\xA8 <b>ALLARME MOTO!</b>\n" + tipoEvento + "\n\n"
                       + posText + segnaleRecap() + batteria()
                       + "\n<i>Premi \xF0\x9F\x94\xB4 Disattiva per fermare le chiamate.</i>";
    if (primoEvento) {
        lastAlarmMsgId    = tgSend(alarmBody, true);   // con tastiera -> bottoni disponibili
        rtcLastAlarmMsgId = lastAlarmMsgId;            // persiste tra sleep
    } else {
        lastAlarmMsgId = rtcLastAlarmMsgId;
        tgEdit(lastAlarmMsgId, "\xF0\x9F\x9A\xA8 <b>ALLARME MOTO!</b>\n"
               "\xE2\x9A\xA1 Nuovo evento: " + tipoEvento + "\n\n"
               + posText + segnaleRecap() + batteria());
    }

    // 3) ZONA LTE (solo primo evento): LIVE location col raggio -> Telegram disegna il
    //    CERCHIO (stile "Dov'e'"). Le statiche NON mostrano il cerchio: ecco perche' prima
    //    vedevi solo un punto. Caption descrittiva SOTTO la mappa.
    if (primoEvento) {
        String rough;
        if (getRoughLocation(rough)) {
            lastAlarmPinId = tgSendLocation(lastLat, lastLon, 3600, min(lastAcc, 1500)); // LIVE + raggio = cerchio
            rtcAlarmPinId  = lastAlarmPinId;
            rtcLteCapId = tgSend("\xF0\x9F\x93\xA1 <i>Zona approssimativa LTE (~" + String(lastAcc)
                                 + "m, cerchio). In attesa del GPS preciso...</i>", false);
        }
    }
    alerted = true; lastMotion = millis();
}

// Sorveglianza dopo l'allarme. NESSUN tempo massimo: resta attivo (GPS+LTE) finche'
// la moto si muove; ogni evento MPU resetta i CALM_PERIOD (10 min). Esce dopo 10 min
// senza movimento o se disarmato. Ri-chiamata su ESCALATION di livello.
// All'arrivo del GPS: cancella pin LTE + caption LTE, apre live location + caption GPS.
void sorvegliaAllarme() {
    gpsOn();
    liveLocMsgId   = rtcLiveLocMsgId;
    lastAlarmPinId = rtcAlarmPinId;
    lastMotion     = millis();
    lastAlarmUpd   = millis() - UPDATE_INTERVAL;  // forza aggiornamento immediato
    lastPoll       = millis();

    while (armed && (millis() - lastMotion < CALM_PERIOD)) {
        unsigned long now = millis();

        // Movimento corrente -> resetta i 10 min e classifica il livello
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        long mag = abs(gx) + abs(gy) + abs(gz);
        if (mag > threshold) lastMotion = now;
        int lvl = livelloMovimento(mag);

        // RI-CHIAMATA: su escalation (livello piu' alto di quelli gia' chiamati),
        // piu' la 2a chiamata ~1 min dopo la prima se c'e' ancora movimento.
        if (!mutedCalls) {
            if (lvl > maxLevelCalled) {
                chiamaTwilio(); lastCall = now; maxLevelCalled = lvl; callsMade++;
            } else if (callsMade == 1 && now - lastCall > CALL_REPEAT && now - lastMotion < CALL_REPEAT) {
                chiamaTwilio(); lastCall = now; callsMade++;
            }
        }

        // GPS / posizione ogni 10s
        if (now - lastAlarmUpd > UPDATE_INTERVAL) {
            String pos;
            if (getFix(pos)) {
                lastGoodPos = pos; lastGpsFixMs = now;
                if (liveLocMsgId == 0) {
                    // PRIMO FIX GPS: cancella pin LTE + caption LTE, apri live location GPS
                    if (lastAlarmPinId) { tgDelete(lastAlarmPinId); lastAlarmPinId = 0; rtcAlarmPinId = 0; }
                    if (rtcLteCapId)    { tgDelete(rtcLteCapId);    rtcLteCapId = 0; }
                    liveLocMsgId    = tgSendLocation(lastLat, lastLon, 3600, 0);  // GPS = live 1h
                    rtcLiveLocMsgId = liveLocMsgId;
                    rtcGpsCapId = tgSend("\xF0\x9F\x9B\xB0 <b>Mappa GPS</b> (aggiornamento ogni 10s)\n"
                                         "Ultimo aggiornamento: <b>" + oraCorrente() + "</b>", false);
                } else {
                    tgMoveLocation(liveLocMsgId, lastLat, lastLon, 0);
                    if (rtcGpsCapId)
                        tgEdit(rtcGpsCapId, "\xF0\x9F\x9B\xB0 <b>Mappa GPS</b> (aggiornamento ogni 10s)\n"
                                            "Ultimo aggiornamento: <b>" + oraCorrente() + "</b>\n" + pos);
                }
                String stato = mutedCalls ? "(chiamate sospese)" : "Movimento in corso...";
                tgEdit(lastAlarmMsgId, "\xF0\x9F\x9A\xA8 <b>ALLARME MOTO!</b>\n" + stato + "\n\n"
                                       + pos + "\n\n" + segnaleRecap() + batteria());
            } else if (rtcGpsCapId && lastGpsFixMs) {
                // GPS perso dopo averlo agganciato: segnala da quanto e' fermo l'aggiornamento
                tgEdit(rtcGpsCapId, "\xF0\x9F\x9B\xB0 <b>Mappa GPS</b>\n\xE2\x9A\xA0 GPS perso - ultimo aggiornamento: "
                                    + daQuando(now - lastGpsFixMs));
            }
            lastAlarmUpd = now;
        }

        if (now - lastPoll > POLL_ARMED) {
            tgCheckCommands(); ntfyCheck();   // permette il disarmo da Telegram
            lastPoll = now;
        }
        delay(50);
    }

    // Le chiamate erano sospese SOLO per questo inseguimento: a fine sessione si
    // riattivano da sole (il prossimo allarme squillera' di nuovo).
    mutedCalls = false;
    alerted = false;

    bool calmExit = armed;   // se ancora armato, e' uscito per i 10 min di calma (non disarmo)

    // Ferma le live location (GPS spento) - sia GPS sia la zona LTE (ora e' live).
    if (rtcLiveLocMsgId) tgStopLive(rtcLiveLocMsgId);
    if (rtcAlarmPinId)   tgStopLive(rtcAlarmPinId);
    if (rtcGpsCapId) {
        if (calmExit)
            tgEdit(rtcGpsCapId, "\xF0\x9F\x9B\xB0 <b>Chiusura live GPS</b>\n"
                                "Sono passati 10 minuti dall'ultimo evento di spostamento.\n"
                                "\xF0\x9F\x93\x8D Ultima posizione registrata alle <b>" + oraCorrente() + "</b>");
        else
            tgEdit(rtcGpsCapId, "\xF0\x9F\x9B\xB0 <b>Ultima posizione GPS</b> (disattivato)\n"
                                "\xE2\x9A\xA0 NON aggiornata - ultimo fix: " + daQuando(millis() - lastGpsFixMs));
    }
    if (calmExit)
        tgSend("\xE2\x8F\xB1 <b>Sono passati 10 minuti dall'ultimo evento di spostamento.</b>\n"
               "Chiusura live GPS. Ultima posizione lasciata in chat.", false);

    // Sessione chiusa: reset di TUTTI gli id (incluse le caption) e dei contatori chiamate.
    rtcLastAlarmMsgId = 0; rtcLiveLocMsgId = 0; rtcAlarmPinId = 0;
    rtcLteCapId = 0; rtcGpsCapId = 0;
    maxLevelCalled = 0; callsMade = 0;
    gpsOff();
}

// ===================== GESTORI WAKEUP =====================

// Wakeup da movimento, allarme ARMATO.
void handleAlarm() {
    Serial.println("ALLARME!");
    // Classifica il tipo di movimento PRIMA del boot (magnitudine ancora presente)
    String tipoEvento = classificaMovimento();
    if (!bootModem(300)) { enterSleep(); return; }
    eseguiAllarme(tipoEvento);   // chiama SUBITO (1a azione), poi notifica Telegram
    sorvegliaAllarme();          // durante la sorveglianza si puo' disarmare da Telegram
    // Se disarmato da comando, dai 1 minuto sveglio (per altri comandi) prima di dormire.
    if (!armed && millis() < awakeUntilMs) return;
    enterSleep();
}

// Wakeup da movimento, allarme DISARMATO.
// 1) interroga lo stato online (l'utente puo' aver armato da remoto)
// 2) invia "Stai spostando?" UNA sola volta; NON la ripete nel ciclo.
//    La rimanda solo dopo 6h ancora disarmato, o dopo uno switch attivo->disattivo.
// 3) entra nel ciclo IGNORA(1min) -> OSSERVA(1min). Vedi enterSleep/handleCheckIn.
void handleMotionDisarmed() {
    Serial.println("Movimento (disarmato)...");

    if (!bootModem(120)) {
        // Niente rete: rientra comunque nella fase IGNORA per non scaricare la batteria
        rtcDisarmPhase = 1;
        enterSleep(); return;
    }

    // Interroga lo stato online (1 sola richiesta/minuto: limita il consumo durante i viaggi)
    tgCheckCommands(); ntfyCheck();

    // *** Armato da remoto MENTRE c'era il movimento che ci ha svegliati ***
    // Questo stesso evento E' un tentativo di furto -> fai partire l'allarme SUBITO.
    if (armed && !theftMode) {
        Serial.println("Armato durante il movimento -> ALLARME immediato");
        eseguiAllarme(classificaMovimento());
        sorvegliaAllarme();
        enterSleep();
        return;
    }
    if (forceAwake || theftMode || liveTracking) return;  // gestito da loop()
    if (millis() < awakeUntilMs) return;  // un comando vuole tenerci svegli -> loop()

    // Sono passate >= 6h dall'ultimo invio? Allora il blocco e' scaduto: reinvia.
    if (rtcCooldownMsgSent && rtcMinSinceMoveMsg >= RESEND_MOVE_MIN) rtcCooldownMsgSent = false;

    // Invia la domanda solo se non gia' inviata in questa "finestra di blocco".
    // Col TIMESTAMP dell'evento (ora di rete letta a freddo): se l'invio fosse in
    // ritardo, sai comunque a che ora la moto e' stata toccata.
    if (!rtcCooldownMsgSent) {
        String msg = "\xF0\x9F\x9A\xB2 <b>Stai spostando tu la moto?</b>\n";
        msg += "\xF0\x9F\x95\x92 <b>Evento rilevato alle " + oraCorrente() + "</b>\n";
        msg += "<i>Allarme disattivato - rilevato movimento</i>\n\n";
        msg += segnaleRecap() + batteria(true);
        msg += "\n\n<i>Usa i bottoni qui sotto o le Shortcut iPhone per attivare l'allarme.</i>";
        tgStatus(msg);                          // porta gia' la tastiera
        rtcCooldownMsgSent = true;
        rtcMinSinceMoveMsg = 0;        // riparte il conteggio delle 6h
    }

    // (Ri)entra nella fase IGNORA: 1 minuto senza ascoltare la MPU
    rtcDisarmPhase = 1;
    enterSleep();
}

// Wakeup da timer: avanzamento ciclo disarmato OPPURE check-in ~4h.
void handleCheckIn() {
    // Fine fase IGNORA -> passa alla fase OSSERVA (MPU riattivata per 1 min).
    if (!rtcArmed && rtcDisarmPhase == 1) {
        Serial.println("Disarmato: fine IGNORA -> OSSERVA (MPU riattivata)");
        rtcDisarmPhase = 2;
        enterSleep();
        return;
    }
    // Fine fase OSSERVA senza movimento = minuto tranquillo -> sleep normale.
    // NB: NON resetto rtcCooldownMsgSent qui, altrimenti rimanderebbe "Stai spostando?"
    // al prossimo urto. Il reinvio avviene solo a 6h (accumulatore) o dopo uno switch.
    if (!rtcArmed && rtcDisarmPhase == 2) {
        Serial.println("Disarmato: minuto tranquillo -> controllo Telegram poi sleep");
        rtcDisarmPhase = 0;
        // A fine ciclo controlla anche i messaggi Telegram pendenti (cambio veicolo,
        // /posizione, ecc.) e rispondi a tutti. Se un comando ci tiene svegli -> loop.
        if (bootModem(120)) { tgCheckCommands(); ntfyCheck(); }
        if (armed || theftMode || liveTracking || forceAwake || millis() < awakeUntilMs) return;
        enterSleep();                 // sleep normale: MPU attiva + check-in orario
        return;
    }

    // Check-in vero ogni ~4h
    Serial.println("Check-in periodico...");
    if (!bootModem(300)) { enterSleep(); return; }
    tgCheckCommands(); ntfyCheck();
    tgStatus("\xF0\x9F\x8F\x8D <b>Check-in</b>\n\n" + statoSistema());
    if (!forceAwake && !theftMode) enterSleep();
}

// Primo boot assoluto
void handleFirstBoot() {
    rtcFirstBoot = false;
    Serial.println("Primo boot...");
    if (!bootModem(300)) { enterSleep(); return; }
    skipOldMessages();  // evita replay di vecchi comandi (es. "calibra")
    tgKeyboard();
    tgStatus("\xF0\x9F\x8F\x8D <b>Antifurto ONLINE</b>\n\n" + statoSistema());
    stayAwakeStart = millis();
}

// ===================== SETUP =====================
void setup() {
    // *** CRITICO: rilascia il hold di MODEM_RESET_PIN impostato prima del sleep ***
    // Deve essere fatto PRIMA di qualsiasi operazione sul modem.
    gpio_hold_dis((gpio_num_t)MODEM_RESET_PIN);

    Serial.begin(115200);

    // Finestra di upload: 8s LED lampeggiante per caricare firmware senza BOOT+RESET
#ifdef BOARD_LED_PIN
    pinMode(BOARD_LED_PIN, OUTPUT);
    for (int i = 0; i < 8; i++) {
        digitalWrite(BOARD_LED_PIN, (i % 2 == 0) ? LED_ON : !LED_ON);
        delay(1000);
    }
    digitalWrite(BOARD_LED_PIN, !LED_ON);
#else
    delay(8000);
#endif
    unsigned long tSer = millis();
    while (!Serial && millis() - tSer < 2000) delay(10);
    setCpuFrequencyMhz(80);
    Serial.println("=== Allarme Moto ===");

    Wire.begin(I2C_SDA, I2C_SCL);
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "MPU6050: OK" : "MPU6050: ERRORE");

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    // A freddo (power-on/reset, RTC azzerata) ricarica la calibrazione del veicolo
    // attivo da NVS. Sul wakeup da deep sleep le soglie sono gia' in RTC.
    if (cause != ESP_SLEEP_WAKEUP_EXT1 && cause != ESP_SLEEP_WAKEUP_TIMER)
        vehApplyActive();

    if (cause == ESP_SLEEP_WAKEUP_EXT1) {
        Serial.println("Wakeup: movimento (EXT1 GPIO6)");
        if (rtcArmed) handleAlarm();
        else          handleMotionDisarmed();
    }
    else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println("Wakeup: timer");
        handleCheckIn();
    }
    else {
        Serial.println("Boot normale");
        if (rtcFirstBoot) {
            handleFirstBoot();
        } else {
            // Reset manuale: riconnetti e mostra stato aggiornato
            if (bootModem(300)) {
                skipOldMessages();  // evita comandi vecchi dopo un reset
                tgKeyboard();
                tgStatus("\xF0\x9F\x94\x84 <b>Sistema riavviato</b>\n\n" + statoSistema());
            }
            stayAwakeStart = millis();
        }
    }

    if (stayAwakeStart == 0) stayAwakeStart = millis();
}

// ===================== LOOP (solo in modalita attiva) =====================
void loop() {
    unsigned long now = millis();

    // Auto-stop della posizione live a tempo (es. /posizione = 10 min)
    if (liveTracking && liveUntilMs && now > liveUntilMs) {
        if (trackLocId) tgStopLive(trackLocId);
        liveTracking = false; liveUntilMs = 0; trackMsgId = 0; trackLocId = 0; trackCapId = 0;
        gpsOff();
        tgStatus("\xE2\x8F\xB1 <b>Posizione live conclusa</b> (10 min). Ultima posizione lasciata in chat.");
    }

    if (theftMode || liveTracking) {
        trackingTick();   // furto (con allarme) o posizione-live on-demand (senza allarme)
    }
    else if (armed) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        long magnitude = abs(gx) + abs(gy) + abs(gz);

        static unsigned long lastDbg = 0;
        if (now - lastDbg > 2000) {
            Serial.printf("giro=%ld soglia=%ld\n", magnitude, threshold);
            lastDbg = now;
        }

        // Movimento da svegli: STESSA logica del wakeup da deep sleep.
        if (magnitude > threshold && !alerted) {
            Serial.println("VIBRAZIONE in loop!");
            String tipo = classificaMovimento();
            eseguiAllarme(tipo);
            sorvegliaAllarme();
            stayAwakeStart = millis();   // resta sveglio un po' dopo l'allarme
            return;
        }
    }
    else {
        if (gpsActive) gpsOff();
        alerted = false;
    }

    unsigned long pollInt = (armed || theftMode) ? POLL_ARMED : POLL_DISARMED;
    if (now - lastPoll > pollInt) {
        tgCheckCommands(); ntfyCheck();
        lastPoll = now;
    }

    // Torna in sleep solo se: non in furto/live, niente forceAwake, finestra base scaduta
    // E nessun comando recente ci tiene svegli (awakeUntilMs, esteso da ogni comando).
    // Dopo uno switch attivo/disattivo la finestra base e' di soli 30s (ripensamento).
    unsigned long awakeWin = justSwitched ? SWITCH_AWAKE_MS : STAY_AWAKE_MS;
    if (!theftMode && !liveTracking && !forceAwake
        && (now - stayAwakeStart > awakeWin) && (now >= awakeUntilMs)) {
        Serial.println("Finestra attiva scaduta, vado in sleep");
        tgStatus("\xF0\x9F\x92\xA4 <b>Sleep attivo</b>\nWakeup su movimento o ogni 4h.\n\n"
                 + segnaleRecap() + batteria(true));
        enterSleep();
    }

    delay(50);
}
