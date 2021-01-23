#include "../include/usbio.hpp"

namespace superpower {

    namespace usbio
    {

        /**
         * @brief  USB communication class
         */
        

        size_t spUSB::read_bus_interrupt(unsigned char endpoint, unsigned int timeout) {
            int len = 0;
            if(this->usb_handle) {
                uint8_t _buf[256];
                libusb_interrupt_transfer(this->usb_handle, endpoint, _buf, sizeof(_buf), &len, timeout);
                if(len > 0) {
                    this->buffer.push(_buf, len);
                }
            }
            return len;
        }

        void spUSB::backend_reading() {
            while(!exited) {
                if(this->usb_handle) {
                    this->read_bus_interrupt();
                }

                struct timeval tv = {0, 1000};
                libusb_handle_events_timeout(this->usb_context, &tv);
            }
        }

        spUSB::spUSB(const int& vid, const int& pid) : vendor_id(vid), product_id(pid) 
        {
            /* Init libusb */
            if(libusb_init(&usb_context) != LIBUSB_SUCCESS) {
                throw "LibUSB initialize failed.\n";
            }

            /* Register hotplug callback */
            if(libusb_hotplug_register_callback(
                this->usb_context, 
                LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED,
                LIBUSB_HOTPLUG_NO_FLAGS, 
                vid, pid, 
                LIBUSB_HOTPLUG_MATCH_ANY, 
                &spUSB::hotplug_callback, this, NULL) != LIBUSB_SUCCESS) 
            {
                throw "Register USB hotplug device arriving callback failed.\n";
            }
            if(libusb_hotplug_register_callback(
                this->usb_context,
                LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
                LIBUSB_HOTPLUG_NO_FLAGS, 
                vid, pid, 
                LIBUSB_HOTPLUG_MATCH_ANY, 
                &spUSB::hotplug_callback, this, NULL) != LIBUSB_SUCCESS)
            {
                throw "Register USB hotplug device leaving callback failed.\n";
            }

            /* Make first trial. */
            if(try_connect()) {
                printf("USB device [0x%04x:0x%04x] is online.\n", this->vendor_id, this->product_id);
            } else {
                printf("USB device [0x%04x:0x%04x] is offline.\n", this->vendor_id, this->product_id);
            }

            /* Start reading thread */
            this->trhead = new boost::thread(std::bind(&spUSB::backend_reading, this));
        }

        spUSB::~spUSB() {
            exited = true;
            libusb_exit(usb_context);
            if(this->trhead) {
                this->trhead->interrupt();
                this->trhead->join();
            }
            this->trhead = NULL;
        }

        bool spUSB::is_working() {
            return !this->exited;
        }

        bool spUSB::try_connect(struct libusb_device *dev) {

            if(this->usb_handle) {
                return true;
            }

            /* Make connecting trial. */
            if(dev) {
                libusb_open(dev, &this->usb_handle);
            } else {
                this->usb_handle = libusb_open_device_with_vid_pid(this->usb_context, this->vendor_id, this->product_id);
            }
            /* Detach kernel deriver and claim interface. */
            if(this->usb_handle) {
                libusb_detach_kernel_driver(this->usb_handle, 0);
                libusb_claim_interface(this->usb_handle, 0);
                return true;
            }
            return false;
        }

        void spUSB::close() {
            if (this->usb_handle) {
                libusb_close(this->usb_handle);
                this->usb_handle = NULL;
            }
        }

        size_t spUSB::available() {
            return this->buffer.avaliable();
        }

        size_t spUSB::read(
            uint8_t *outbuf, size_t size, 
            const uint8_t sof, const uint8_t eof, 
            std::function<bool(uint8_t*, size_t)> check ) 
        {
            return this->buffer.get_by(outbuf, size, sof, eof, check);
        }

        size_t spUSB::read(uint8_t *outbuf, size_t size) {
            return this->buffer.get(outbuf, size, false);
        }

        int spUSB::write(uint8_t * data, size_t size, int& len, unsigned char endpoint, unsigned int timeout) {
            if(this->usb_handle) {
                // uint8_t _buf[256];
                return libusb_interrupt_transfer(this->usb_handle, endpoint, data, size, &len, timeout);
            }
            return LIBUSB_ERROR_IO;
        }

    } // threadsafe
    
}
