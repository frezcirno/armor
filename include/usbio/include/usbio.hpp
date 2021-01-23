#ifndef __USBIO_H
#define __USBIO_H

#include <stdio.h>
#include <termio.h>
#include <time.h>

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <iostream>
#include <thread>
#include <mutex>

#include <functional>
#include <libusb-1.0/libusb.h>
#include <boost/thread.hpp>

#include "../include/utility.hpp"

namespace superpower {

    namespace usbio
    {
        /**
         * @brief  USB communication class
         * @note
        Typical usage:
            sp::usbio::spUSB usb(0x0477, 0x5620);
            while(true) {
                size_t asize;
                frame_t frame;
                if( (asize=usb.avaliable()) > 0 ) {
                    uint8_t buf[1024];
                    size_t size = usb.read(buf, asize, 0xCC, 0x0A, [](uint8_t* _buf, size_t _size)->bool {
                        return _size == 62;
                    });

                    if(size > 0) {
                        memcpy((uint8_t*)&frame, buf, size);
                        for(int i=0; i<15; i++) {
                            printf("%f ", frame.data[i]);
                        }
                    }
                    printf("\n");
                }
                usleep(1000);
            }
         */
        class spUSB {
        private:
            libusb_context             *usb_context = NULL;     /*!< USB context */
            libusb_device_handle       *usb_handle = NULL;      /*!< USB device handle */
            int                         vendor_id = -1;         /*!< USB vendor id */
            int                         product_id = -1;        /*!< USB product id */

            utility::Buffer             buffer;                 /*!< Inner data buffer */
            boost::thread              *trhead = NULL;          /*!< Thread for USB reading */
            bool                        exited = false;         /*!< Flag for thread exiting */

        private:
            /**
             * @brief  USB hotplug callback function
             * @param ctx: usb context
             * @param dev: activing device
             * @param event: plug event, can be @arg LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED or @arg LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT
             * @param userdata: user data given when registe callback
             */
            static int hotplug_callback(
                struct libusb_context *ctx, 
                struct libusb_device *dev,
                libusb_hotplug_event event,
                void *user_data)
            {
                struct libusb_device_descriptor desc;
                libusb_get_device_descriptor(dev, &desc);
                if (LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED == event) {
                    if(((spUSB*)user_data)->try_connect(dev)) {
                        printf("USB device [0x%04x:0x%04x] connected.\n", desc.idVendor, desc.idProduct);
                    } else {
                        ((spUSB*)user_data)->usb_handle = NULL;
                        printf("Could not open USB device [0x%04x:0x%04x].\n", desc.idVendor, desc.idProduct);
                    }
                } else if (LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT == event) {
                    ((spUSB*)user_data)->close();
                    printf("USB device [0x%04x:0x%04x] disconnected.\n", desc.idVendor, desc.idProduct);
                }
                return 0;
            }

            /**
             * @brief  Reding by USB interrupt mode in *thread*.
             * @param endpoint: reading endpoint
             * @param timeout: reading timeout each trial [ms]
             */
            // TODO: Make interface accessiable.
            size_t read_bus_interrupt(unsigned char endpoint=0x81, unsigned int timeout=1);

            /**
             * @brief  Thread function for reading in background.
             */
            void backend_reading();

        public:
            /**
             * @brief  Constructor
             * @param vid: USB vendor id
             * @param pid: UBS product id
             * @note  Register callbacks and start backend reading thread.
             */
            spUSB(const int& vid, const int& pid);

            /**
             * @brief  Destructor
             * @note  Stop thread and close USB interface.
             */
            ~spUSB();

        public:
            /**
             * @brief  USB interface status.
             * @retval true=thread is working, false=thread is exiting(usually class destructed).
             */
            bool is_working();
                
            /**
             * @brief  Try to claim USB device.
             * @param dev: @ref libusb_device, make convenience for callback fucntion, or NULL when using id from class self parameters.
             * @retval if USB connected, if so, USB handler will not be NULL
             */
            bool try_connect(struct libusb_device *dev=NULL);

            /**
             * @brief  Close USB interface
             */
            void close();

            /**
             * @brief  Get available reading bytes.
             */
            size_t available();

            /**
             * @brief  Read data by searching frame SOF and EOF, you are suggested to give check medthod.
             * @param outbuf: array for output data
             * @param size: max supposed size of bytes
             * @param sof: start of frame byte
             * @param eof: end of frame byte
             * @param check: frame validation function
             */
            size_t read(uint8_t *outbuf, size_t size, const uint8_t sof, const uint8_t eof,
                std::function<bool(uint8_t*, size_t)> check = [](uint8_t* buf, size_t size)->bool{return true;});

            /**
             * @brief  Get newest readable bytes.
             */
            size_t read(uint8_t *outbuf, size_t size);

            /**
             * @brief  Writing by USB interrupt mode in synchronous mode.
             * @param data: data for sending
             * @param size: length of data
             * @param len: transfered data length
             * @param endpoint: reading endpoint
             * @param timeout: reading timeout each trial [ms]
             * @retval 0=success, otherwise for usb errors.
             */
            // TODO: Make interface accessiable.
            int write(uint8_t * data, size_t size, int& len, unsigned char endpoint=0x01, unsigned int timeout=1);
        };
        
    } // usbio
    
}

#endif /* __USBIO_H */

