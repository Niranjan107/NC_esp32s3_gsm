/**
 * @file net_link.h
 * @brief The whole contract between the connectivity layer and the application
 *        layer (see docs/ARCHITECTURE.md).
 *
 * The application layer (MQTT publish, store-and-forward, FOTA) has exactly one
 * question about the network: "can I send right now?". It must never know
 * WHETHER that link is WiFi, GSM, or a gateway - otherwise the layer cannot be
 * copied into the GSM product without editing it.
 *
 * So the connectivity component registers itself here at startup, and the
 * application asks net_link_is_up(). Today wifi_sta registers one link and the
 * behaviour is identical to calling ncle_wifi_sta_is_connected() directly.
 * Later the GSM component registers one instead - and no application code
 * changes at all.
 *
 * Two slots exist so a future device can have both (GSM + WiFi): is_up() is
 * then true when EITHER is up, and net_link_active() reports which one is
 * carrying traffic. Nothing else about dual-mode is built yet.
 */
#ifndef NCLE_NET_LINK_H
#define NCLE_NET_LINK_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** How many links may register (GSM + WiFi is the eventual maximum). */
#define NET_LINK_MAX 2

/**
 * @brief One way of reaching the internet, as the application layer sees it.
 *
 * The struct is stored BY POINTER, so it must have static storage duration -
 * a link registered from the stack would dangle. Every connectivity component
 * defines it as a `static const`.
 */
typedef struct {
    const char *name;       /**< "wifi" / "gsm" - shown in diag and logs. */
    bool (*is_up)(void);    /**< Is this link usable right now? Must not block. */

    /**
     * Optional. Write a JSON OBJECT describing this link into buf, e.g.
     * {"connected":true,"ssid":"X","rssi":-40} for WiFi, or
     * {"connected":true,"sim":"ok","signal":-71} for GSM. Returns the number of
     * bytes written. May be NULL - diag then reports {} for this link.
     *
     * This is what lets `diag` stay transport-neutral: it prints whatever the
     * active link says about itself, so a GSM product reports SIM and signal
     * with no change to diag at all.
     */
    int (*status_json)(char *buf, size_t size);

    /**
     * Optional. Has this link been GIVEN credentials - not "is it up now"?
     * WiFi answers "is an SSID saved"; GSM would answer "is an APN set".
     *
     * This separates "nobody intends to use the network here" from "the network
     * is down at the moment", and store-and-forward turns on exactly the first
     * one: credentials present means somebody wants the cloud, so a reading
     * taken during an outage must be kept. No credentials means the site is
     * app-only and buffering would just fill flash for nothing.
     *
     * May be NULL - the link is then assumed provisioned, i.e. buffering stays
     * on, which is the safe default.
     */
    bool (*is_provisioned)(void);
} net_link_t;

/**
 * @brief Register a link. Called once by the connectivity component at startup.
 *
 * INPUT:
 * @param link  statically-allocated descriptor (see the note above)
 *
 * OUTPUT:
 * @return true if registered; false if `link` is malformed or all
 *         NET_LINK_MAX slots are taken (logged as an error).
 */
bool net_link_register(const net_link_t *link);

/**
 * @brief Can anything send right now?
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return true if ANY registered link reports itself up. false when no link is
 *         registered at all - a build with no connectivity layer behaves like a
 *         permanently offline device, which is correct, not an error.
 */
bool net_link_is_up(void);

/**
 * @brief Which link is carrying traffic - for diag and logs only.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return the name of the first registered link that is up, "none" if a link is
 *         registered but down, or "unconfigured" if nothing registered.
 *         Never NULL, so it is always safe to print.
 */
const char *net_link_active(void);

/**
 * @brief Ask the active link to describe itself, for diag.
 *
 * INPUT:
 * @param buf   destination for a JSON object
 * @param size  size of buf
 *
 * OUTPUT:
 * @return bytes written. Writes "{}" when no link is up or the active link
 *         provides no status_json, so the caller can always splice the result
 *         straight into a larger JSON document.
 */
int net_link_status_json(char *buf, size_t size);

/**
 * @brief Has any link been given credentials?
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return true if any registered link reports itself provisioned, or if a link
 *         is registered but does not implement is_provisioned (assume yes - the
 *         safe answer, since it keeps store-and-forward on).
 *         false only when a link is registered and says it has no credentials.
 *
 * With NO link registered this returns false: a build with no connectivity
 * layer can never deliver, so buffering would fill flash for nothing.
 */
bool net_link_is_provisioned(void);

#ifdef __cplusplus
}
#endif

#endif /* NCLE_NET_LINK_H */
