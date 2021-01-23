#include "../include/utility.hpp"

namespace superpower {

    namespace utility {
        double getSecond(void) {
            /* Publish joint values */
            struct timeval tv;
            gettimeofday(&tv, NULL);
            return (double) tv.tv_sec + (double) tv.tv_usec / 1000000.f;
        }


        void Buffer::__init__(void) {
            buffer = new uint8_t[size_main];
            memset(buffer, 0x00, size_main);
            sub_buffer = new uint8_t[size_second];
            memset(sub_buffer, 0x00, size_second);
        }

        size_t Buffer::push(const uint8_t *inbuf, size_t size) {

            try {

                if (size > size_main)
                    throw Buffer::BufferOverflowException();

                boost::unique_lock<boost::shared_mutex> lock(mutex_);

                size_t lsize = size_main - ptr_c;
                size_t hsize = (lsize >= size) ? 0 : size - lsize;
                lsize = size - hsize;

                memcpy(&buffer[ptr_c], inbuf, lsize);
                memcpy(buffer, &inbuf[lsize], hsize);

                ptr_c = (ptr_c + lsize + hsize) % size_main;

                return lsize + hsize;
            } catch (const Buffer::BufferOverflowException &e) {
                throw e;
            }
            catch (const std::exception &e) {
                printf("%s\n", e.what());
                return 0;
            }
        }

        size_t Buffer::get(uint8_t *outbuf, size_t max_size, bool copy) {
            /**
             * Only get size bytes near to ptr_c.
             *  Assume memory as:
             *      (A) Head .... [ptr_e] ..([ptr_b]size_y+[ptr_a]size_x).. ..[ptr_c] ..  Tail
             *      (B) Head ..[ptr_a](size_x).. [ptr_c] .... [ptr_e] ..[ptr_b](size_y)..  Tail
             */
            try {
                boost::shared_lock<boost::shared_mutex> lock(mutex_);

                size_t size_x = 0, size_y = 0;
                int size = ptr_c - ptr_e;

                /* (A) */
                if (size >= 0) {
                    size_x = (size > max_size) ? max_size : size;
                }
                    /* (B) */
                else {
                    size = size_main + size;
                    size = (size > max_size) ? max_size : size;

                    if (ptr_c > max_size) {
                        size_x = max_size;
                    } else {
                        size_x = ptr_c;
                        size_y = size - size_x;
                    }
                }

                /* Get ptr */
                int ptr_a = -1, ptr_b = -1;
                ptr_a = ptr_c - size_x;
                if (ptr_a == 0 && size > 0) {
                    ptr_b = size_main - size_y;
                } else {
                    ptr_b = ptr_a;
                }
                /* Output buffer */
                memcpy(outbuf, &buffer[ptr_b], size_y);
                memcpy(&outbuf[size_y], &buffer[ptr_a], size_x);

                if (!copy) {
                    this->ptr_e = this->ptr_c;
                }
                return size_y + size_x;

            } catch (const std::exception &e) {
                printf("%s\n", e.what());
                return 0;
            }
        }

        size_t Buffer::avaliable(void) {
            boost::shared_lock<boost::shared_mutex> lock(mutex_);
            int size = ptr_c - ptr_e;
            size += (size < 0) ? size_main : 0;
            return size;
        }

        void Buffer::offset(const size_t &size) {
            size_t asize = avaliable();
            size_t off = (size > asize) ? asize : size;

            boost::unique_lock<boost::shared_mutex> lock(mutex_);
            ptr_e = (ptr_e + off) % (size_main);
        }


        size_t Buffer::get_by(uint8_t *outbuf, size_t size, const uint8_t sof, const uint8_t eof,
                              std::function<bool(uint8_t *, size_t)> check
        ) {

            /* Get a copy of avaliable bytes */
            uint8_t *_buf = outbuf;
            size_t asize = get(_buf, size, true);

            /* Search for valid bytes */
            size_t gsize = 0;
            /* Start and end of possible frame */
            int ptr_s = -1, ptr_d = -1;
            /* Finding sof */
            for (int i = asize - 1; i >= 0; --i) {
                if (_buf[i] == eof) {
                    ptr_d = i;
                    /* Finding eof that after this sof */
                    for (int j = ptr_d - 1; j >= 0; --j) {
                        if (_buf[j] == sof) {
                            ptr_s = j;
                            /* Check this possible frame */
                            gsize = ptr_d - ptr_s + 1;
                            if (gsize > 0) {
                                if (check(&_buf[ptr_s], gsize)) {
                                    /* Output valid bytes array */
                                    memcpy(outbuf, &_buf[ptr_s], gsize);
                                    offset(ptr_d + 1);
                                    return gsize;
                                }
                            }
                        }
                    }
                }
            }
            return 0;
        }

        void Buffer::show(void) {
            for (int i = 0; i < this->size_main; ++i) {
                printf("0x%02x ", this->buffer[i]);
                if ((i + 1) % 8 == 0) printf("\n");
            }
            std::cout << std::endl;
        }

        size_t Buffer::get_current_ptr(void) {
            return this->ptr_c;
        }

        size_t Buffer::get_last_ptr(void) {
            return this->ptr_e;
        }


    } // utility

}