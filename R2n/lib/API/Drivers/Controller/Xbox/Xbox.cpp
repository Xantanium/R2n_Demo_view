#include <Arduino.h>
#include <cstdint>
#include <USBHost_t36.h>  // Read this header first for key info
#include "Xbox.h"

#define print   USBHost::print_
#define println USBHost::println_

// #define DEBUG_JOYSTICK
#ifdef  DEBUG_JOYSTICK
#define DBGPrintf USBHDBGSerial.printf
#else
#define DBGPrintf(...)
#endif

// PID/VID to joystick mapping - Only the XBOXOne is used to claim the USB interface directly,
// The others are used after claim-hid code to know which one we have and to use it for
// doing other features.
XboxController::product_vendor_mapping_t XboxController::pid_vid_mapping[] = {
  { 0x045e, 0x02ea, XBOXONE, false }, { 0x045e, 0x02dd, XBOXONE, false },
  { 0x045e, 0x0719, XBOX360, false},
  { 0x2563, 0x0575, XBOX360, true},
  { 0x24C6, 0x542A, XBOX360, false},
  // VendorID = 24C6, ProductID = 542A,
  { 0x045e, 0x028E, SWITCH, false},  // Switch? // -> hack to detect as a HID
  // Xbox 360 with a generic hid packet of Switch -> generic wireless HID protocol
};



//-----------------------------------------------------------------------------
void XboxController::init()
{
  contribute_Pipes(mypipes, sizeof(mypipes) / sizeof(Pipe_t));
  contribute_Transfers(mytransfers, sizeof(mytransfers) / sizeof(Transfer_t));
  contribute_String_Buffers(mystring_bufs, sizeof(mystring_bufs) / sizeof(strbuf_t));
  driver_ready_for_device(this);
  USBHIDParser::driver_ready_for_hid_collection(this);
  BluetoothController::driver_ready_for_bluetooth(this);
}

//-----------------------------------------------------------------------------
XboxController::joytype_t XboxController::mapVIDPIDtoJoystickType(uint16_t idVendor, uint16_t idProduct, bool exclude_hid_devices)
{
  for (uint8_t i = 0; i < (sizeof(pid_vid_mapping) / sizeof(pid_vid_mapping[0])); i++) {
    if ((idVendor == pid_vid_mapping[i].idVendor) && (idProduct == pid_vid_mapping[i].idProduct)) {
      println("Match PID/VID: ", i, DEC);
      if (exclude_hid_devices && pid_vid_mapping[i].hidDevice) return UNKNOWN;
      return pid_vid_mapping[i].joyType;
    }
  }
  return UNKNOWN;   // Not in our list
}

//*****************************************************************************
// Some simple query functions depend on which interface we are using...
//*****************************************************************************

uint16_t XboxController::idVendor()
{
  if (device != nullptr) return device->idVendor;
  if (mydevice != nullptr) return mydevice->idVendor;
  return 0;
}

uint16_t XboxController::idProduct()
{
  if (device != nullptr) return device->idProduct;
  if (mydevice != nullptr) return mydevice->idProduct;
  return 0;
}

const uint8_t *XboxController::manufacturer()
{
  if ((device != nullptr) && (device->strbuf != nullptr)) return &device->strbuf->buffer[device->strbuf->iStrings[strbuf_t::STR_ID_MAN]];
  //if ((btdevice != nullptr) && (btdevice->strbuf != nullptr)) return &btdevice->strbuf->buffer[btdevice->strbuf->iStrings[strbuf_t::STR_ID_MAN]];
  if ((mydevice != nullptr) && (mydevice->strbuf != nullptr)) return &mydevice->strbuf->buffer[mydevice->strbuf->iStrings[strbuf_t::STR_ID_MAN]];
  return nullptr;
}

const uint8_t *XboxController::product()
{
  if ((device != nullptr) && (device->strbuf != nullptr)) return &device->strbuf->buffer[device->strbuf->iStrings[strbuf_t::STR_ID_PROD]];
  if ((mydevice != nullptr) && (mydevice->strbuf != nullptr)) return &mydevice->strbuf->buffer[mydevice->strbuf->iStrings[strbuf_t::STR_ID_PROD]];
  if (btdevice != nullptr) return remote_name_;
  return nullptr;
}

const uint8_t *XboxController::serialNumber()
{
  if ((device != nullptr) && (device->strbuf != nullptr)) return &device->strbuf->buffer[device->strbuf->iStrings[strbuf_t::STR_ID_SERIAL]];
  if ((mydevice != nullptr) && (mydevice->strbuf != nullptr)) return &mydevice->strbuf->buffer[mydevice->strbuf->iStrings[strbuf_t::STR_ID_SERIAL]];
  return nullptr;
}


static uint8_t rumble_counter = 0;

bool XboxController::setRumble(uint8_t lValue, uint8_t rValue, uint8_t timeout)
{
  // Need to know which joystick we are on.  Start off with XBox support - maybe need to add some enum value for the known
  // joystick types.
  rumble_lValue_ = lValue;
  rumble_rValue_ = rValue;
  rumble_timeout_ = timeout;

  switch (joystickType_) {
    default:
      break;
    case XBOX360:
      txbuf_[0] = 0x00;
      txbuf_[1] = 0x01;
      txbuf_[2] = 0x0F;
      txbuf_[3] = 0xC0;
      txbuf_[4] = 0x00;
      txbuf_[5] = lValue;
      txbuf_[6] = rValue;
      txbuf_[7] = 0x00;
      txbuf_[8] = 0x00;
      txbuf_[9] = 0x00;
      txbuf_[10] = 0x00;
      txbuf_[11] = 0x00;
      if (!queue_Data_Transfer(txpipe_, txbuf_, 12, this)) {
        println("XBox360 rumble transfer fail");
      }
      return true;
    case SWITCH:
      memset(txbuf_, 0, 10);  // make sure it is cleared out
      txbuf_[0] = 0x80;
      txbuf_[1] = 0x92;
      txbuf_[3] = 0x31;
      txbuf_[8] = 0x10; // Command

      // Now add in subcommand data:
      // Probably do this better soon
      txbuf_[9 + 0] = rumble_counter++; //
      txbuf_[9 + 1] = 0x80;
      txbuf_[9 + 2] = 0x00;
      txbuf_[9 + 3] = 0x40;
      txbuf_[9 + 4] = 0x40;
      txbuf_[9 + 5] = 0x80;
      txbuf_[9 + 6] = 0x00;
      txbuf_[9 + 7] = 0x40;
      txbuf_[9 + 8] = 0x40;

      if (lValue != 0) {
        txbuf_[9 + 5] = 0x08;
        txbuf_[9 + 6] = lValue;
      } else if (rValue != 0) {
        txbuf_[9 + 5] = 0x10;
        txbuf_[9 + 6] = rValue;
      }

      if (!queue_Data_Transfer(txpipe_, txbuf_, 18, this)) {
        println("switch rumble transfer fail");
        Serial.printf("Switch Rumble transfer fail\n");
      }
      return true;
  }
  return false;
}


bool XboxController::setLEDs(uint8_t lr, uint8_t lg, uint8_t lb)
{
  // Need to know which joystick we are on.  Start off with XBox support - maybe need to add some enum value for the known
  // joystick types.
  //Serial.printf("::setLEDS(%x %x %x)\n", lr, lg, lb);
  if ((leds_[0] != lr) || (leds_[1] != lg) || (leds_[2] != lb)) {
    leds_[0] = lr;
    leds_[1] = lg;
    leds_[2] = lb;

    switch (joystickType_) {
      case XBOX360:
        // 0: off, 1: all blink then return to before
        // 2-5(TL, TR, BL, BR) - blink on then stay on
        // 6-9() - On
        // ...
        txbuf_[1] = 0x00;
        txbuf_[2] = 0x08;
        txbuf_[3] = 0x40 + lr;
        txbuf_[4] = 0x00;
        txbuf_[5] = 0x00;
        txbuf_[6] = 0x00;
        txbuf_[7] = 0x00;
        txbuf_[8] = 0x00;
        txbuf_[9] = 0x00;
        txbuf_[10] = 0x00;
        txbuf_[11] = 0x00;
        if (!queue_Data_Transfer(txpipe_, txbuf_, 12, this)) {
          println("XBox360 set leds fail");
        }
        return true;
      case SWITCH:
        memset(txbuf_, 0, 10);  // make sure it is cleared out
        txbuf_[0] = 0x80;
        txbuf_[1] = 0x92;
        txbuf_[3] = 0x31;
        txbuf_[8] = 0x01; // Command

        // Now add in subcommand data:
        // Probably do this better soon
        txbuf_[9 + 0] = rumble_counter++; //
        txbuf_[9 + 1] = 0x00;
        txbuf_[9 + 2] = 0x01;
        txbuf_[9 + 3] = 0x40;
        txbuf_[9 + 4] = 0x40;
        txbuf_[9 + 5] = 0x00;
        txbuf_[9 + 6] = 0x01;
        txbuf_[9 + 7] = 0x40;
        txbuf_[9 + 8] = 0x40;

        txbuf_[9 + 9] = 0x30; // LED Command
        txbuf_[9 + 10] = lr;
        println("Switch set leds: driver? ", (uint32_t)driver_, HEX);
        print_hexbytes((uint8_t*)txbuf_, 20);
        if (!queue_Data_Transfer(txpipe_, txbuf_, 20, this)) {
          println("switch set leds fail");
        }

      case XBOXONE:
      default:
        return false;
    }
  }
  return false;
}


//*****************************************************************************
// Support for Joysticks that Use HID data.
//*****************************************************************************

hidclaim_t XboxController::claim_collection(USBHIDParser *driver, Device_t *dev, uint32_t topusage)
{
  // only claim Desktop/Joystick and Desktop/Gamepad
  if (topusage != 0x10004 && topusage != 0x10005 && topusage != 0x10008) return CLAIM_NO;
  // only claim from one physical device
  if (mydevice != NULL && dev != mydevice) return CLAIM_NO;

  // Also don't allow us to claim if it is used as a standard usb object (XBox...)
  if (device != nullptr) return CLAIM_NO;

  mydevice = dev;
  collections_claimed++;
  anychange = true; // always report values on first read
  driver_ = driver; // remember the driver.
  driver_->setTXBuffers(txbuf_, nullptr, sizeof(txbuf_));
  connected_ = true;    // remember that hardware is actually connected...

  joystickType_ = mapVIDPIDtoJoystickType(mydevice->idVendor, mydevice->idProduct, false);
  DBGPrintf("XboxController::claim_collection joystickType_=%d\n", joystickType_);
  additional_axis_usage_page_ = 0x09;
  additional_axis_usage_start_ = 0x21;
  additional_axis_usage_count_ = 5;
  axis_change_notify_mask_ = 0x3ff; // Start off assume only the 10 bits...

  DBGPrintf("Claim Additional axis: %x %x %d\n", additional_axis_usage_page_, additional_axis_usage_start_, additional_axis_usage_count_);
  return CLAIM_REPORT;
}

void XboxController::disconnect_collection(Device_t *dev)
{
  if (--collections_claimed == 0) {
    mydevice = NULL;
    driver_ = nullptr;
    axis_mask_ = 0;
    axis_changed_mask_ = 0;
  }
}

void XboxController::hid_input_begin(uint32_t topusage, uint32_t type, int lgmin, int lgmax)
{
  // TODO: set up translation from logical min/max to consistent 16 bit scale
}

void XboxController::hid_input_data(uint32_t usage, int32_t value)
{
  DBGPrintf("joystickType_=%d\n", joystickType_);
  DBGPrintf("Joystick: usage=%X, value=%d\n", usage, value);
  uint32_t usage_page = usage >> 16;
  usage &= 0xFFFF;
  if (usage_page == 9 && usage >= 1 && usage <= 32) {
    uint32_t bit = 1 << (usage - 1);
    if (value == 0) {
      if (buttons & bit) {
        buttons &= ~bit;
        anychange = true;
        button_change = true;
      }
    } else {
      if (!(buttons & bit)) {
        buttons |= bit;
        anychange = true;
        button_change = true;
      }
    }
    DBGPrintf("Siddharth Debug-> up==9 line number 476 reached \n");
  } else if (usage_page == 1 && usage >= 0x30 && usage <= 0x39) {
    uint32_t i = usage - 0x30;
    axis_mask_ |= (1 << i);
    if (axis[i] != value) {
      axis[i] = value;
      axis_changed_mask_ |= (1 << i);
      if (axis_changed_mask_ & axis_change_notify_mask_)
        anychange = true;
    }
  } else if (usage_page == additional_axis_usage_page_) {
    //DBGPrintf("UP: usage_page=%x usage=%x User: %x %d\n", usage_page, usage, user_buttons_usage_start, user_buttons_count_);
    if ((usage >= additional_axis_usage_start_) && (usage < (additional_axis_usage_start_ + additional_axis_usage_count_))) {
      // We are in the user range.
      uint16_t usage_index = usage - additional_axis_usage_start_ + STANDARD_AXIS_COUNT;
      if (usage_index < (sizeof(axis) / sizeof(axis[0]))) {
        if (axis[usage_index] != value) {
          axis[usage_index] = value;
          if (usage_index > 63) usage_index = 63; // don't overflow our mask
          axis_changed_mask_ |= ((uint64_t)1 << usage_index);   // Keep track of which ones changed.
          if (axis_changed_mask_ & axis_change_notify_mask_)
            anychange = true; // We have changes...
        }
        axis_mask_ |= ((uint64_t)1 << usage_index);   // Keep record of which axis we have data on.
      }
      //DBGPrintf("UB: index=%x value=%x\n", usage_index, value);
    }

  } else {
    DBGPrintf("UP: usage_page=%x usage=%x add: %x %x %d\n", usage_page, usage, additional_axis_usage_page_, additional_axis_usage_start_, additional_axis_usage_count_);

  }
  // TODO: hat switch?
}

void XboxController::hid_input_end()
{
  if (anychange) {
    joystickEvent = true;
    //    isConnected = true;
  }
}

bool XboxController::hid_process_out_data(const Transfer_t *transfer)
{
  //DBGPrintf("XboxController::hid_process_out_data\n");
  return true;
}

bool XboxController::hid_process_in_data(const Transfer_t *transfer)
{
  uint8_t *pb = (uint8_t *)transfer->buffer;
  if (!transfer->buffer || *pb == 1) return false; // don't do report 1
  Serial.printf("hid_process_in_data %x %u:", transfer->buffer, transfer->length);
  uint8_t cnt = transfer->length;
  if (cnt > 16) cnt = 16;
  while (cnt--) Serial.printf(" %02x", *pb++);
  Serial.printf("\n");

  return false;
}

bool XboxController::hid_process_control(const Transfer_t *transfer) {
  Serial.printf("USBHIDParser::control msg: %x %x : %x %u :", transfer->setup.word1, transfer->setup.word2, transfer->buffer, transfer->length);
  if (transfer->buffer) {
    uint16_t cnt = transfer->length;
    if (cnt > 16) cnt = 16;
    uint8_t *pb = (uint8_t*)transfer->buffer;
    while (cnt--) Serial.printf(" %02x", *pb++);
  }
  Serial.printf("\n");
  send_Control_packet_active_ = false;
  return false;
}

void XboxController::joystickDataClear() {
  joystickEvent = false;
  anychange = false;
  axis_changed_mask_ = 0;
  axis_mask_ = 0;
  button_change = false;

}

static  uint8_t xboxone_start_input[] = {0x05, 0x20, 0x00, 0x01, 0x00};
static  uint8_t xbox360w_inquire_present[] = {0x08, 0x00, 0x0F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//static  uint8_t switch_start_input[] = {0x19, 0x01, 0x03, 0x07, 0x00, 0x00, 0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10};
static  uint8_t switch_start_input[] = {0x80, 0x02};
bool XboxController::claim(Device_t *dev, int type, const uint8_t *descriptors, uint32_t len)
{
  println("XboxController claim this=", (uint32_t)this, HEX);

  // Don't try to claim if it is used as USB device or HID device
  if (mydevice != NULL) return false;
  if (device != nullptr) return false;

  // Try claiming at the interface level.
  if (type != 1) return false;
  print_hexbytes(descriptors, len);

  XboxController::joytype_t jtype = mapVIDPIDtoJoystickType(dev->idVendor, dev->idProduct, true);
  println("Jtype=", (uint8_t)jtype, DEC);
  if (jtype == UNKNOWN)
    return false;

  // XBOX 360 wireless... Has 8 interfaces.  4 joysticks (1, 3, 5, 7) and 4 headphones assume 2,4,6, 8...
  // Shows data for #1 only...
  // Also they have some unknown data type we need to ignore between interface and end points.
  //  0  1  2  3  4  5  6  7  8 *9 10  1  2  3  4  5 *6  7  8  9 20  1  2  3  4  5  6  7  8
  // 09 04 00 00 02 FF 5D 81 00 14 22 00 01 13 81 1D 00 17 01 02 08 13 01 0C 00 0C 01 02 08

  if (len < 9 + 7 + 7) return false;

  // Some common stuff for both XBoxs
  uint32_t count_end_points = descriptors[4];
  if (count_end_points < 2) return false;
  if (descriptors[5] != 0xff) return false; // bInterfaceClass, 3 = HID
  rx_ep_ = 0;
  uint32_t txep = 0;
  uint8_t rx_interval = 0;
  uint8_t tx_interval = 0;
  rx_size_ = 0;
  tx_size_ = 0;
  uint32_t descriptor_index = 9;
  if (descriptors[descriptor_index + 1] == 0x22)  {
    if (descriptors[descriptor_index] != 0x14) return false;
    descriptor_index += descriptors[descriptor_index];
  }
  while ((rx_ep_ == 0) || txep == 0) {
    print("  Index:", descriptor_index, DEC);

    if (descriptor_index >= len) return false;
    if ((descriptors[descriptor_index] == 7) && (descriptors[descriptor_index + 1] == 5)) {
      if ((descriptors[descriptor_index + 3] == 3)
          && (descriptors[descriptor_index + 4] <= 64)
          && (descriptors[descriptor_index + 5] == 0)) {
        // have a bulk EP size
        if (descriptors[descriptor_index + 2] & 0x80 ) {
          rx_ep_ = descriptors[descriptor_index + 2];
          rx_size_ = descriptors[descriptor_index + 4];
          rx_interval = descriptors[descriptor_index + 6];
        } else {
          txep = descriptors[descriptor_index + 2];
          tx_size_ = descriptors[descriptor_index + 4];
          tx_interval = descriptors[descriptor_index + 6];
        }
      }
    }
    descriptor_index += descriptors[descriptor_index];
  }
  if ((rx_ep_ == 0) || (txep == 0)) return false;
  print("XboxController, rx_ep_=", rx_ep_ & 15);
  print("(", rx_size_);
  print("), txep=", txep);
  print("(", tx_size_);
  println(")");
  rxpipe_ = new_Pipe(dev, 3, rx_ep_ & 15, 1, rx_size_, rx_interval);
  if (!rxpipe_) return false;
  txpipe_ = new_Pipe(dev, 3, txep, 0, tx_size_, tx_interval);
  if (!txpipe_) {
    //free_Pipe(rxpipe_);
    return false;
  }
  rxpipe_->callback_function = rx_callback;
  queue_Data_Transfer(rxpipe_, rxbuf_, rx_size_, this);

  txpipe_->callback_function = tx_callback;

  if (jtype == XBOX360) {
    queue_Data_Transfer(txpipe_, xbox360w_inquire_present, sizeof(xbox360w_inquire_present), this);
    connected_ = 0;
    isConnected = false;
  } else if (jtype == SWITCH) {
    queue_Data_Transfer(txpipe_, switch_start_input, sizeof(switch_start_input), this);
    connected_ = true;
    isConnected = true;
  }
  memset(axis, 0, sizeof(axis));
  joystickType_ = jtype;
  DBGPrintf("   XboxController::claim joystickType_ %d\n", joystickType_);
  return true;
}

void XboxController::control(const Transfer_t *transfer)
{
}


/************************************************************/
//  Interrupt-based Data Movement
/************************************************************/

void XboxController::rx_callback(const Transfer_t *transfer)
{
  if (!transfer->driver) return;
  ((XboxController *)(transfer->driver))->rx_data(transfer);
}

void XboxController::tx_callback(const Transfer_t *transfer)
{
  if (!transfer->driver) return;
  ((XboxController *)(transfer->driver))->tx_data(transfer);
}

typedef struct {
  uint8_t type;
  uint8_t const_0;
  uint16_t id;
  uint16_t buttons;
  int16_t axis[6];
} xbox1data20_t;

typedef struct {
  uint8_t state;
  uint8_t id_or_type;
  uint16_t controller_status;
  uint16_t unknown;
  uint16_t buttons;
  uint8_t lt;
  uint8_t rt;
  int16_t axis[4];
} xbox360data_t;

typedef struct {
  uint8_t state;
  uint8_t id_or_type;
  uint8_t buttons_h;
  uint8_t buttons_l;
  uint8_t lt;
  uint8_t rt;
  int16_t axis[4];
  uint8_t test;
} switchdataUSB_t;

static const uint8_t xbox_axis_order_mapping[] = {3, 4, 0, 1, 2, 5};

void XboxController::rx_data(const Transfer_t *transfer)
{
#ifdef  DEBUG_JOYSTICK
  print("XboxController::rx_data (", joystickType_, DEC);
  print("): ");
  print_hexbytes((uint8_t*)transfer->buffer, transfer->length);
#endif
  if (joystickType_ == XBOX360) {
    // First byte appears to status - if the byte is 0x8 it is a connect or disconnect of the controller.
    xbox360data_t  *xb360d = (xbox360data_t *)transfer->buffer;
    if (xb360d->state == 0x08) {
      if (xb360d->id_or_type != connected_) {
        connected_ = xb360d->id_or_type;  // remember it...
        if (connected_) {
          println("XBox360w - Connected type:", connected_, HEX);
          // rx_ep_ should be 1, 3, 5, 7 for the wireless convert to 2-5 on led
          setLEDs(2 + rx_ep_ / 2); // Right now hard coded to first joystick...
          isConnected = true;
        } else {
          println("XBox360w - disconnected");
          isConnected = false;
        }
      }
    } else if ((xb360d->id_or_type == 0x00) && (xb360d->controller_status & 0x1300)) {
      println("XBox360w - controllerStatus: ", xb360d->controller_status, HEX);
    } else if (xb360d->id_or_type == 0x01) { // Lets only process report 1.
      //const uint8_t *pbuffer = (uint8_t*)transfer->buffer;
      //for (uint8_t i = 0; i < transfer->length; i++) DBGPrintf("%02x ", pbuffer[i]);
      //DBGPrintf("\n");

      if (buttons != xb360d->buttons) {
        buttons = xb360d->buttons;
        anychange = true;
        button_change = true;
      }
      axis_mask_ = 0x3f;
      axis_changed_mask_ = 0; // assume none for now

      for (uint8_t i = 0; i < 4; i++) {
        if (axis[i] != xb360d->axis[i]) {
          axis[i] = xb360d->axis[i];
          axis_changed_mask_ |= (1 << i);
          anychange = true;
        }
      }
      // the two triggers show up as 4 and 5
      if (axis[4] != xb360d->lt) {
        axis[4] = xb360d->lt;
        axis_changed_mask_ |= (1 << 4);
        anychange = true;
      }

      if (axis[5] != xb360d->rt) {
        axis[5] = xb360d->rt;
        axis_changed_mask_ |= (1 << 5);
        anychange = true;
      }

      if (anychange) {
        joystickEvent = true;
      }
    }
  } else if (joystickType_ == SWITCH) {
    switchdataUSB_t  *switchd = (switchdataUSB_t *)transfer->buffer;

    // do something to detect disconnection -Siddharth

    //    isConnected = switchd->state;
    //    test = switchd->test;
    //    if (switchd->state == 0x08) {
    //      if (switchd->id_or_type != connected_) {
    //        connected_ = switchd->id_or_type;  // remember it...
    //        if (connected_) {
    //          println("XBox360w - Connected type:", connected_, HEX);
    //          // rx_ep_ should be 1, 3, 5, 7 for the wireless convert to 2-5 on led
    //          setLEDs(2 + rx_ep_ / 2); // Right now hard coded to first joystick...
    ////          isConnected = true;
    //        } else {
    //          println("XBox360w - disconnected");
    ////          isConnected = false;
    //        }
    //      }
    //    }

    uint16_t cur_buttons = (switchd->buttons_h << 8) | switchd->buttons_l;
    if (buttons != cur_buttons) {
      buttons = cur_buttons;
      anychange = true;
      button_change = true;
    }
    axis_mask_ = 0x3f;
    axis_changed_mask_ = 0; // assume none for now

    for (uint8_t i = 0; i < 4; i++) {
      if (axis[i] != switchd->axis[i]) {
        axis[i] = switchd->axis[i];
        axis_changed_mask_ |= (1 << i);
        anychange = true;
      }
    }
    // the two triggers show up as 4 and 5
    if (axis[4] != switchd->lt) {
      axis[4] = switchd->lt;
      axis_changed_mask_ |= (1 << 4);
      anychange = true;
    }

    if (axis[5] != switchd->rt) {
      axis[5] = switchd->rt;
      axis_changed_mask_ |= (1 << 5);
      anychange = true;
    }

    if (anychange) {
      joystickEvent = true;
    }
  }

  // hack to detect first connection change later - Siddharth
  if (isFirst && axis_changed_mask_ == 0) {
    isConnected = false;
  } else if (isFirst && axis_changed_mask_ != 0) {
    isConnected = true;
    isFirst = false;
  }
  // hack end
  queue_Data_Transfer(rxpipe_, rxbuf_, rx_size_, this);
}

void XboxController::tx_data(const Transfer_t *transfer)
{
}

void XboxController::disconnect()
{
  axis_mask_ = 0;
  axis_changed_mask_ = 0;
  connected_ = false;
  isConnected = false;
  // TODO: free resources
}

bool XboxController::mapNameToJoystickType(const uint8_t *remoteName)
{
  // Sort of a hack, but try to map the name given from remote to a type...
  if (strncmp((const char *)remoteName, "Wireless Controller", 19) == 0) {
    DBGPrintf("  XboxController::mapNameToJoystickType %s - set to PS4\n", remoteName);
    joystickType_ = PS4;
  } else if (strncmp((const char *)remoteName, "PLAYSTATION(R)3", 15) == 0) {
    DBGPrintf("  XboxController::mapNameToJoystickType %x %s - set to PS3\n", (uint32_t)this, remoteName);
    joystickType_ = PS3;
  } else if (strncmp((const char *)remoteName, "Navigation Controller", 21) == 0) {
    DBGPrintf("  XboxController::mapNameToJoystickType %x %s - set to PS3\n", (uint32_t)this, remoteName);
    joystickType_ = PS3;
  } else if (strncmp((const char *)remoteName, "Motion Controller", 17) == 0) {
    DBGPrintf("  XboxController::mapNameToJoystickType %x %s - set to PS3 Motion\n", (uint32_t)this, remoteName);
    joystickType_ = PS3_MOTION;
  } else if (strncmp((const char *)remoteName, "Xbox Wireless", 13) == 0) {
    DBGPrintf("  XboxController::mapNameToJoystickType %x %s - set to XBOXONE\n", (uint32_t)this, remoteName);
    joystickType_ = XBOXONE;
  } else {
    DBGPrintf("  XboxController::mapNameToJoystickType %s - Unknown\n", remoteName);
  }
  DBGPrintf("  Joystick Type: %d\n", joystickType_);
  return true;
}

bool XboxController::getButtonClick(ButtonEnum x) {
    int index = 0;
	uint32_t val = x;
    unsigned long curr_millis = millis();
    while (val >>= 1) {
        index++;
    }

    if ((buttons & x) && (curr_millis - button_millis[index] > 300)) {
        button_millis[index] = curr_millis;
        return true;
    }

    return false;
}

bool XboxController::getButtonPress(ButtonEnum x) {
  return (buttons & x);
}

int XboxController::getAnalogHat(JoyEnum x) {
  return axis[x];
}

bool XboxController::isActive() {
  return connected_;
}
