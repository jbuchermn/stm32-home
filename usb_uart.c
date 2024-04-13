// Source: https://github.com/roddehugo/bluepill-serial

#include "usb_uart.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/usbd.h>

#define USB_DRV st_usbfs_v1_usb_driver
#define USB_IRQ NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR usb_lp_can_rx0_isr

#define GPIO_MODE_OUTPUT GPIO_MODE_OUTPUT_2_MHZ
#define GPIO_CONF_OUTPUT GPIO_CNF_OUTPUT_PUSHPULL

static char serialno[9] = {0};

static usbd_device *usbdev = 0;
static uint16_t usbd_configuration = 0;
static uint8_t usbd_control_buffer[256];

__attribute__((always_inline)) inline void assertm(bool condition,
                                                   const char *msg) {
  if (!__builtin_expect(condition, 1)) {
    static volatile const char *reason;
    reason = msg;
    (void)reason;
    __asm__ volatile("bkpt");
    while (1)
      ;
  }
}

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

static void cdcacm_data_rx_cb(usbd_device *dev, uint8_t ep) {
  (void)ep;

  static char buf[CDCACM_PACKET_SIZE + 1];
  uint16_t len = usbd_ep_read_packet(dev, 0x01, buf, CDCACM_PACKET_SIZE);

  /* Echo buffer back to TX line. */
  cdcacm_write_now(buf, len);
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

void cdcacm_init(void) {
  serialno_read(serialno);

  usb_data_line_reset();

  usbdev = usbd_init(&USB_DRV, &device, &config, usb_strings,
                     sizeof(usb_strings) / sizeof(char *), usbd_control_buffer,
                     sizeof(usbd_control_buffer));
  assert(usbdev);
  usbd_register_set_config_callback(usbdev, cdcacm_set_config);
  nvic_enable_irq(USB_IRQ);
}

int cdcacm_get_configuration(void) { return usbd_configuration; }

const char *cdcacm_get_serialno(void) { return serialno; }

void cdcacm_write_now(const char *buf, int len) {
  assert(usbdev);
  assert(len <= CDCACM_PACKET_SIZE);
  int i;
  const char *p = buf;
  const char *const end = buf + len;
  while (p < end) {
    i = 0;
    while (p + i < end) {
      if (p[i] == '\r' || p[i] == '\n')
        break;
      i++;
    }
    if (i > 0) {
      while (usbd_ep_write_packet(usbdev, 0x81, p, i) == 0)
        ;
      p += i;
    }
    if (*p == '\r' || *p == '\n') {
      while (usbd_ep_write_packet(usbdev, 0x81, "\r\n", 2) == 0)
        ;
      p++;
    }
  }
}
