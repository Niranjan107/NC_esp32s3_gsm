/**
 * @file net_link.c
 * @brief Fixed-size registry of network links (see net_link.h).
 *
 * Deliberately tiny: a fixed array, no allocation, no mutex. Registration
 * happens once during startup, before any reader exists; after that the table
 * is read-only, so the only shared state is the count - written once, read
 * afterwards. The array is `volatile` so a reader on the other core cannot see
 * a cached stale count.
 */
#include "net_link.h"
#include "esp_log.h"

static const char *TAG = "NET_LINK";

static const net_link_t *volatile s_links[NET_LINK_MAX];
static volatile int s_count;

bool net_link_register(const net_link_t *link)
{
    if (!link || !link->is_up || !link->name) {
        ESP_LOGE(TAG, "register: malformed link descriptor");
        return false;
    }
    if (s_count >= NET_LINK_MAX) {
        ESP_LOGE(TAG, "register: no free slot for '%s' (max %d)",
                 link->name, NET_LINK_MAX);
        return false;
    }

    s_links[s_count] = link;
    s_count++;                      /* published last: a reader sees a full slot */
    ESP_LOGI(TAG, "link registered: %s", link->name);
    return true;
}

bool net_link_is_up(void)
{
    for (int i = 0; i < s_count; i++) {
        if (s_links[i]->is_up()) {
            return true;
        }
    }
    return false;
}

int net_link_status_json(char *buf, size_t size)
{
    for (int i = 0; i < s_count; i++) {
        if (s_links[i]->is_up() && s_links[i]->status_json) {
            return s_links[i]->status_json(buf, size);
        }
    }
    /* No link up, or the active one offers no detail: an empty OBJECT, never an
     * empty string - the caller splices this into a bigger JSON document. */
    if (buf && size >= 3) {
        buf[0] = '{'; buf[1] = '}'; buf[2] = '\0';
        return 2;
    }
    return 0;
}

bool net_link_is_provisioned(void)
{
    for (int i = 0; i < s_count; i++) {
        /* A link that does not implement is_provisioned is assumed to have
         * credentials. That keeps store-and-forward ON, which is the safe way
         * to be wrong: buffering a reading that could not be delivered costs
         * flash, while not buffering one that could have been loses it. */
        if (!s_links[i]->is_provisioned || s_links[i]->is_provisioned()) {
            return true;
        }
    }
    return false;
}

bool net_link_any_enabled(void)
{
    for (int i = 0; i < s_count; i++) {
        if (!s_links[i]->is_enabled || s_links[i]->is_enabled()) {
            return true;
        }
    }
    return false;
}

const char *net_link_active(void)
{
    if (s_count == 0) {
        return "unconfigured";
    }
    for (int i = 0; i < s_count; i++) {
        if (s_links[i]->is_up()) {
            return s_links[i]->name;
        }
    }
    return "none";
}
