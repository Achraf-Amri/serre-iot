#pragma once
#include <stdbool.h>

/* Appelee pour chaque message recu. suffix = "/cmd", "/config/set", ... */
typedef void (*net_rx_cb_t)(const char *suffix, const char *data, int len);

void      net_set_rx_cb(net_rx_cb_t cb);
void      net_start(void);
bool      net_wifi_ok(void);
bool      net_mqtt_ok(void);
bool      net_take_new_connection(void);   /* true une fois apres chaque (re)connexion MQTT */
long long net_epoch(void);                 /* heure UTC en secondes, 0 si pas encore synchronisee */
int       net_publish(const char *suffix, const char *payload, int qos, bool retain);