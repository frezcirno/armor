#ifndef __UTILITY_H
#define __UTILITY_H

#include <stdio.h>
#include <termio.h>
#include <time.h>

#include <iostream>
#include <vector>
#include <sys/time.h>

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>

#include <boost/thread.hpp>

namespace superpower {
    /**
     * \033[ colors
     * [0;31m	Red
     * [1;31m	Bold Red
     * [0;32m	Green
     * [1;32m	Bold Green
     * [0;33m	Yellow
     * [01;33m	Bold Yellow
     * [0;34m	Blue
     * [1;34m	Bold Blue
     * [0;35m	Magenta
     * [1;35m	Bold Magenta
     * [0;36m	Cyan
     * [1;36m	Bold Cyan
     * [0m	Reset
     */

    namespace utility
    {


        /**
         *  Smart byte type buffer queue.
         */
        class Buffer {
        private:
            uint8_t                         *buffer;                /*!< Main buffer. */
            const size_t                    size_main = 2048;
            size_t                          ptr_c = 0;              /*!< Current pointer */
            size_t                          ptr_e = 0;              /*!< Last pointer */
            
            const size_t                    size_second = 1024;
            uint8_t                         *sub_buffer;            /*!< Second buffer, may be not used. */
            // int32_t                         overed = 0;             /*!< How many bytes overloaded in sub-buffer from main-buffer */

            mutable boost::shared_mutex     mutex_;

            /**
             *  Distribute space for pointers.
             */
            void __init__(void);

        public:
            
            /**
             *  Using default buffer size.
             */
            Buffer(void):size_main(2048), size_second(1024) {
                __init__();
            }

            /**
             *  Using user buffer size.
             */
            Buffer(const size_t& size):size_main(size), size_second(size/2) {
                if(size<= 0) {
                    throw "Empty buffer size";
                }
                __init__();
            }

            ~Buffer() {
                if(buffer) delete buffer;
                if(sub_buffer) delete sub_buffer;
            }

        public:
            class BufferOverflowException : public std::exception {
                public:
                    BufferOverflowException() = default;
                    virtual ~BufferOverflowException() = default;
                    const char* what() const throw() {
                        return "Buffer overflow.";
                    }
            };

        public:
            /**
             *  Push bytes to end of buffer.
             *  \param inbuf: input buffer
             *  \param size: size of input bytes 
             */
            size_t push(const uint8_t *inbuf, size_t size);

            /**
             *  Get newest bytes with max reading size. 
             *  \param outbuf: output buffer
             *  \param size: max size of output bytes 
             *  \param copy: if using copy mode, which means not flush pointer but only get avaliable bytes
             *  \exception Buffer::BufferOverflowException()
             *      \note Data will not be pushed into buffer if this occurses.
             */
            size_t get(uint8_t *outbuf, size_t size, bool copy=false);

            /**
             *  Get newest bytes with SOF/EOF filter, always get FIRST valid bytes array.
             *  \param outbuf: output buffer
             *  \param size: max size of output bytes 
             *  \param sof: start of frame byte
             *  \param eof: end of frame byte
             *  \retval actually length of got bytes
             */
            size_t get_by(uint8_t *outbuf, size_t size, const uint8_t sof, const uint8_t eof,
                std::function<bool(uint8_t*, size_t)> check = [](uint8_t* buf, size_t size)->bool{return true;});

            /**
             *  Get unread bytes size.
             */
            size_t avaliable(void);

            /**
             *  Get pointer position.
             */
            size_t get_current_ptr(void);
            size_t get_last_ptr(void);

            /**
             *  Discard all unread bytes.
             */
            void flush(void);

            /**
             *  Offset last pointer to refresh avaliable data.
             */
            void offset(const size_t& size);

            /**
             *  Display all bytes in the current this->buffer array.
             */
            void show(void);

        };

    } // utility
    
}

#endif /* __UTILITY_H */

