// Source: https://github.com/roddehugo/bluepill-serial
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/usbd.h>

#include "cdcacm.h"
#include "main.h"
#include "util.h"

#define NOTIF_PACKET_SIZE 16
#define CDCACM_PACKET_SIZE 64
#define CDCACM_RETRY 10

#define USB_DRV st_usbfs_v1_usb_driver
#define USB_IRQ NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR usb_lp_can_rx0_isr

#define GPIO_MODE_OUTPUT GPIO_MODE_OUTPUT_2_MHZ
#define GPIO_CONF_OUTPUT GPIO_CNF_OUTPUT_PUSHPULL

static char serialno[9] = {0};

static usbd_device *usbdev = 0;
static uint16_t usbd_configuration = 0;
static uint8_t usbd_control_buffer[256];

static const char *usb_strings[] = {
    "Hugo Rodde http://roddehugo.com",
    "Bluepill Serial",
    serialno,
};

static const struct usb_device_descriptor device = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x16c0,  /* ID for discrimination by textual name. */
    .idProduct = 0x05e1, /* For CDC-ACM class devices (modems). */
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = NOTIF_PACKET_SIZE,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x01,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = CDCACM_PACKET_SIZE,
        .bInterval = 1,
    },
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x81,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = CDCACM_PACKET_SIZE,
        .bInterval = 1,
    }};

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
    .header =
        {
            .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
            .bcdCDC = 0x0110,
        },
    .call_mgmt =
        {
            .bFunctionLength =
                sizeof(struct usb_cdc_call_management_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
            .bmCapabilities = 0,
            .bDataInterface = 1,
        },
    .acm =
        {
            .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_ACM,
            .bmCapabilities = 0,
        },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
    }};

static const struct usb_interface_descriptor comm_iface[] = {
    {.bLength = USB_DT_INTERFACE_SIZE,
     .bDescriptorType = USB_DT_INTERFACE,
     .bInterfaceNumber = 0,
     .bAlternateSetting = 0,
     .bNumEndpoints = 1,
     .bInterfaceClass = USB_CLASS_CDC,
     .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
     .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
     .iInterface = 0,

     .endpoint = comm_endp,

     .extra = &cdcacm_functional_descriptors,
     .extralen = sizeof(cdcacm_functional_descriptors)}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = data_endp,
}};

static const struct usb_iface_assoc_descriptor cdcacm_assoc = {
    .bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
    .bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
    .bFirstInterface = 0,
    .bInterfaceCount = 2,
    .bFunctionClass = USB_CLASS_CDC,
    .bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
    .bFunctionProtocol = USB_CDC_PROTOCOL_AT,
    .iFunction = 0,
};

static const struct usb_interface ifaces[] = {{
                                                  .num_altsetting = 1,
                                                  .iface_assoc = &cdcacm_assoc,
                                                  .altsetting = comm_iface,
                                              },
                                              {
                                                  .num_altsetting = 1,
                                                  .altsetting = data_iface,
                                              }};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = sizeof(ifaces) / sizeof(ifaces[0]),
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,

    .interface = ifaces,
};

static void cdcacm_set_modem_state(usbd_device *dev, int iface, bool dsr,
                                   bool dcd) {
    char buf[10];
    struct usb_cdc_notification *notif = (void *)buf;
    /* We echo signals back to host as notification. */
    notif->bmRequestType = 0xA1;
    notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
    notif->wValue = 0;
    notif->wIndex = iface;
    notif->wLength = 2;
    buf[8] = (dsr ? 2 : 0) | (dcd ? 1 : 0);
    buf[9] = 0;
    usbd_ep_write_packet(dev, 0x82 + iface, buf, 10);
}

static enum usbd_request_return_codes cdcacm_control_request(
    usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
    void (**complete)(usbd_device *dev, struct usb_setup_data *req)) {
    (void)dev;
    (void)buf;
    (void)complete;

    switch (req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
        cdcacm_set_modem_state(dev, req->wIndex, true, true);
        return USBD_REQ_HANDLED;
    case USB_CDC_REQ_SET_LINE_CODING:
        if (*len < sizeof(struct usb_cdc_line_coding))
            return USBD_REQ_NOTSUPP;
        return USBD_REQ_HANDLED;
    }
    return USBD_REQ_NOTSUPP;
}

/* RX and TX */
static char __rx_buf[CDCACM_BUFSIZE];
static char __tx_buf[CDCACM_BUFSIZE];

static uint16_t __rx_at;
static uint16_t __rx_len;
static uint16_t __tx_at;
static uint16_t __tx_len;

static void cdcacm_data_rx_cb(usbd_device *dev, uint8_t ep) {
    /* Overflow - read to slow */
    if (CDCACM_BUFSIZE < __rx_len + CDCACM_PACKET_SIZE) {
        __rx_at = 0;
        __rx_len = 0;
    }

    __rx_len +=
        usbd_ep_read_packet(dev, 0x01, __rx_buf + __rx_len, CDCACM_PACKET_SIZE);
}

static void cdcacm_set_config(usbd_device *dev, uint16_t wValue) {
    /* Store configuration value. */
    usbd_configuration = wValue;

    /* Setup interface. */
    usbd_ep_setup(dev, 0x01, USB_ENDPOINT_ATTR_BULK, CDCACM_PACKET_SIZE,
                  cdcacm_data_rx_cb);
    usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_BULK, CDCACM_PACKET_SIZE, 0);
    usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_INTERRUPT, NOTIF_PACKET_SIZE, 0);

    usbd_register_control_callback(
        dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, cdcacm_control_request);

    /* Notify the host that DCD is asserted.
     * Allows the use of /dev/tty* devices on BSD/macOS. */
    cdcacm_set_modem_state(dev, 0, true, true);
}

static char *serialno_read(char *buf) {
    volatile uint32_t *DEVICE_ID = (volatile uint32_t *)0x1FFFF7E8;
    const uint32_t uid = *DEVICE_ID + *(DEVICE_ID + 1) + *(DEVICE_ID + 2);

    /* Fetch serial number from chip's unique ID. */
    for (int i = 0; i < 8; i++)
        buf[7 - i] = ((uid >> (4 * i)) & 0xF) + '0';

    for (int i = 0; i < 8; i++)
        if (buf[i] > '9')
            buf[i] += 'A' - '9' - 1;
    buf[8] = 0;

    return buf;
}

static void usb_data_line_reset(void) {
    /* This is a somewhat common cheap hack to trigger device re-enumeration
     * on startup.  Assuming a fixed external pullup on D+, (For USB-FS)
     * setting the pin to output, and driving it explicitly low effectively
     * "removes" the pullup.  The subsequent USB init will "take over" the
     * pin, and it will appear as a proper pullup to the host. */
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT, GPIO_CONF_OUTPUT, GPIO12);
    gpio_clear(GPIOA, GPIO12);

    /* The magic delay is somewhat arbitrary, no guarantees on USBIF
     * compliance here, but "it works" in most places. */
    for (int i = 0; i < 0x8000; ++i)
        __asm__ volatile("nop");
}

void USB_ISR(void) {
    assert(usbdev);
    usbd_poll(usbdev);
}

/* API */

int cdcacm_init(void) {
    __rx_at = 0;
    __rx_len = 0;
    __tx_at = 0;
    __tx_len = 0;

    serialno_read(serialno);
    usb_data_line_reset();

    usbdev = usbd_init(&USB_DRV, &device, &config, usb_strings,
                       sizeof(usb_strings) / sizeof(char *),
                       usbd_control_buffer, sizeof(usbd_control_buffer));
    assert(usbdev);
    usbd_register_set_config_callback(usbdev, cdcacm_set_config);
    nvic_enable_irq(USB_IRQ);

    return 0;
}

int cdcacm_main(void) {
    if (!usbd_configuration)
        return -1;

    if (__tx_len > __tx_at) {
        int len = __tx_len - __tx_at;
        if (len > CDCACM_PACKET_SIZE)
            len = CDCACM_PACKET_SIZE;

        __tx_at += usbd_ep_write_packet(usbdev, 0x81, __tx_buf + __tx_at, len);
    }

    if (__tx_at == __tx_len) {
        __tx_at = 0;
        __tx_len = 0;
    }

    return 0;
}

uint16_t cdcacm_len_read(void) { return __rx_len - __rx_at; }
uint16_t cdcacm_len_line(void) {
    for (int i = 0; i < __rx_len - __rx_at; i++) {
        if (__rx_buf[__rx_at + i] == '\r')
            return i + 1;
    }
    return 0;
}

uint16_t cdcacm_read(char **buf, uint16_t max) {
    *buf = __rx_buf + __rx_at;
    uint16_t res = __rx_len - __rx_at < max ? __rx_len - __rx_at : max;
    __rx_at += res;
    if (__rx_at == __rx_len) {
        __rx_at = __rx_len = 0;
    }
    return res;
}

int cdcacm_write(char *buf, uint16_t len) {
    /* Overflow - write too fast */
    if (__tx_len + len > CDCACM_BUFSIZE) {
        return -1;
    }

    for (int i = 0; i < len; i++) {
        __tx_buf[__tx_len + i] = buf[i];
    }

    __tx_len += len;
    return 0;
}

void cdcacm_write_hex(int val, uint8_t len) {
    if (val < 0) {
        cdcacm_write("-", 1);
        val *= -1;
    }

    for (int i = len - 1; i >= 0; i--) {
        char buf = (val >> (4 * i)) & 0xF;
        if (buf < 10) {
            buf = '0' + buf;
        } else {
            buf = 'A' + (buf - 0xA);
        }
        cdcacm_write(&buf, 1);
    }
}
