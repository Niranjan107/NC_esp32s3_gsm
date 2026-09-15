/* Local broker credentials - NOT committed (see .gitignore).
 * Copied from mqtt_secrets.h.example. */
#ifndef MQTT_SECRETS_H
#define MQTT_SECRETS_H

/* --- Nitara UAT server (MQTT over WebSocket Secure, nginx -> broker) --- */
#define NCLE_MQTT_URI   "wss://uat-mqtt.nitara.co.in:443/mqtt"
#define NCLE_MQTT_USER  "devops"
#define NCLE_MQTT_PASS  "QAZplm"

/* --- Own HiveMQ Cloud test cluster (swap back by uncommenting) ---
#define NCLE_MQTT_URI   "mqtts://a7fe44294f064a4eabca39cba9d5535f.s1.eu.hivemq.cloud:8883"
#define NCLE_MQTT_USER  "nitara_device"
#define NCLE_MQTT_PASS  "Fastrack143$"
*/

#endif /* MQTT_SECRETS_H */
