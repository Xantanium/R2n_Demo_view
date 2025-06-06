#pragma once
#include <Stream.h>
#include <USBHost_t36.h>
#include <cstddef>
#include <cstring>
#include "controller_enums.h"

class XboxController : public USBDriver, public USBHIDInput, public BTHIDInput {
public:
    XboxController(USBHost& host) { init(); }

    uint16_t idVendor();
    uint16_t idProduct();

    const uint8_t* manufacturer();
    const uint8_t* product();
    const uint8_t* serialNumber();
    operator bool()
    {
        return (((device != nullptr) || (mydevice != nullptr || (btdevice != nullptr)))
            && connected_); // override as in both USBDriver and in USBHIDInput
    }

    bool available() { return joystickEvent; }
    void joystickDataClear();
    uint32_t getButtons() { return buttons; }
    int getAxis(uint32_t index) { return (index < (sizeof(axis) / sizeof(axis[0]))) ? axis[index] : 0; }
    uint64_t axisMask() { return axis_mask_; }
    uint64_t axisChangedMask() { return axis_changed_mask_; }
    uint64_t axisChangeNotifyMask() { return axis_change_notify_mask_; }
    void axisChangeNotifyMask(uint64_t notify_mask) { axis_change_notify_mask_ = notify_mask; }

    // set functions functionality depends on underlying joystick.
    bool setRumble(uint8_t lValue, uint8_t rValue, uint8_t timeout = 0xff);
    // setLEDs on PS4(RGB), PS3 simple LED setting (only uses lb)
    bool setLEDs(uint8_t lr, uint8_t lg, uint8_t lb); // sets Leds,
    bool inline setLEDs(uint32_t leds)
    {
        return setLEDs((leds >> 16) & 0xff, (leds >> 8) & 0xff,
            leds & 0xff); // sets Leds - passing one arg for all leds
    }
    enum {
        STANDARD_AXIS_COUNT = 10,
        ADDITIONAL_AXIS_COUNT = 54,
        TOTAL_AXIS_COUNT = (STANDARD_AXIS_COUNT + ADDITIONAL_AXIS_COUNT)
    };
    typedef enum { UNKNOWN = 0, PS3, PS4, XBOXONE, XBOX360, PS3_MOTION, SpaceNav, SWITCH } joytype_t;
    joytype_t joystickType() { return joystickType_; }

    bool getButtonClick(ButtonEnum x);
    bool getButtonPress(ButtonEnum x);
    int getAnalogHat(JoyEnum x);
    bool isActive();
    bool isConnected = false;
    int test = 0;
    bool button_change = false;
    bool isFirst = true;

    unsigned long lastPacket = 0;

protected:
    // From USBDriver
    virtual bool claim(Device_t* device, int type, const uint8_t* descriptors, uint32_t len);
    virtual void control(const Transfer_t* transfer);
    virtual void disconnect();

    // From USBHIDInput
    virtual hidclaim_t claim_collection(USBHIDParser* driver, Device_t* dev, uint32_t topusage);
    virtual void hid_input_begin(uint32_t topusage, uint32_t type, int lgmin, int lgmax);
    virtual void hid_input_data(uint32_t usage, int32_t value);
    virtual bool hid_process_control(const Transfer_t* transfer);
    virtual void hid_input_end();
    virtual void disconnect_collection(Device_t* dev);
    virtual bool hid_process_out_data(const Transfer_t* transfer);
    virtual bool hid_process_in_data(const Transfer_t* transfer);

    // Bluetooth data
    //    virtual bool claim_bluetooth(BluetoothController *driver, uint32_t bluetooth_class,
    //    uint8_t *remoteName); virtual bool process_bluetooth_HID_data(const uint8_t *data,
    //    uint16_t length); virtual void release_bluetooth(); virtual bool remoteNameComplete(const
    //    uint8_t *remoteName); virtual void connectionComplete(void);

    joytype_t joystickType_ = UNKNOWN;

private:
    // Class specific
    void init();
    USBHIDParser* driver_ = nullptr;
    BluetoothController* btdriver_ = nullptr;

    joytype_t mapVIDPIDtoJoystickType(uint16_t idVendor, uint16_t idProduct, bool exclude_hid_devices);
    //    bool transmitPS4UserFeedbackMsg();
    //    bool transmitPS3UserFeedbackMsg();
    //    bool transmitPS3MotionUserFeedbackMsg();
    bool mapNameToJoystickType(const uint8_t* remoteName);
    uint8_t remote_name_;
    bool anychange = false;
    volatile bool joystickEvent = false;
    uint32_t buttons = 0;
    unsigned long button_millis[16] = { 0 };
    uint32_t is_clicked_buttons = 0;
    int axis[TOTAL_AXIS_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint64_t axis_mask_ = 0; // which axis have valid data
    uint64_t axis_changed_mask_ = 0;
    uint64_t axis_change_notify_mask_ = 0x3ff; // assume the low 10 values only.

    uint16_t additional_axis_usage_page_ = 0;
    uint16_t additional_axis_usage_start_ = 0;
    uint16_t additional_axis_usage_count_ = 0;

    // State values to output to Joystick.
    uint8_t rumble_lValue_ = 0;
    uint8_t rumble_rValue_ = 0;
    uint8_t rumble_timeout_ = 0;
    uint8_t leds_[3] = { 0, 0, 0 };
    uint8_t connected_ = 0; // what type of device if any is connected xbox 360...

    // Used by HID code
    uint8_t collections_claimed = 0;

    // Used by USBDriver code
    static void rx_callback(const Transfer_t* transfer);
    static void tx_callback(const Transfer_t* transfer);
    void rx_data(const Transfer_t* transfer);
    void tx_data(const Transfer_t* transfer);

    Pipe_t mypipes[3] __attribute__((aligned(32)));
    Transfer_t mytransfers[7] __attribute__((aligned(32)));
    strbuf_t mystring_bufs[1];

    uint8_t rx_ep_ = 0; // remember which end point this object is...
    uint16_t rx_size_ = 0;
    uint16_t tx_size_ = 0;
    Pipe_t* rxpipe_;
    Pipe_t* txpipe_;
    uint8_t rxbuf_[64]; // receive circular buffer
    uint8_t txbuf_[64]; // buffer to use to send commands to joystick
    volatile bool send_Control_packet_active_;
    // Mapping table to say which devices we handle
    typedef struct {
        uint16_t idVendor;
        uint16_t idProduct;
        joytype_t joyType;
        bool hidDevice;
    } product_vendor_mapping_t;
    static product_vendor_mapping_t pid_vid_mapping[];
};
