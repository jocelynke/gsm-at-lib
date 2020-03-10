/**
 * \file            gsm_ll_template.c
 * \brief           Low-level communication with GSM device template
 */

/*
 * Copyright (c) 2020 Tilen MAJERLE
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of GSM-AT library.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 * Version:         $_version_$
 */
#include "system/gsm_ll.h"
#include "gsm/gsm.h"
#include "gsm/gsm_mem.h"
#include "gsm/gsm_input.h"
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

static uint8_t initialized = 0;
static int fd_serial = -1;
static pthread_t rx_thread;

#if !defined(GSM_MEM_SIZE)
#define GSM_MEM_SIZE                    0x1000
#endif /* !defined(GSM_MEM_SIZE) */

static void* uart_thread_func(void* param);

/**
 * \brief           Send data to GSM device, function called from GSM stack when we have data to send
 * \param[in]       data: Pointer to data to send
 * \param[in]       len: Number of bytes to send
 * \return          Number of bytes sent
 */
static size_t
send_data(const void* data, size_t len) {
#ifdef GSM_CFG_AT_ECHO
#endif
    ssize_t written = write(fd_serial, data, len);
    if(written < 0)
        return 0;
    return written;  /* Return number of bytes actually sent to AT port */
}

static gsmr_t
set_baudrate(uint32_t baudrate){
    if(fd_serial == -1)
        return gsmERR;

    struct termios config;
    if(tcgetattr(fd_serial, &config) < 0)
        return gsmERR;

    //TODO : cette config est par défaut dans notre cas, il faudrait prévoir une méthode publique pour que l'utilisateur puisse la définir
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON | IUTF8);

    config.c_oflag = 0;

    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | ECHOE | ECHOK | ECHOCTL | ECHOKE);

    config.c_cflag &= ~(CSIZE | PARENB | HUPCL | CMSPAR);
    config.c_cflag |= CS8;

    memset(config.c_cc, 0, sizeof(config.c_cc));
    config.c_cc[VMIN]  = 1;

    speed_t speed;
    switch (baudrate)
    {
        case 115200: speed = B115200; break;
        case 57600: speed = B57600; break;
        case 38400: speed = B38400; break;
        case 19200: speed = B19200; break;
        case 9600:  speed = B9600;  break;
        case 4800:  speed = B4800;  break;
        case 2400:  speed = B2400;  break;
        case 1200:  speed = B1200;  break;
        default:    speed = B9600;  break;  /* default is 9600 */
    }

    if(cfsetspeed(&config, speed) != 0)   //on set le baudrate en in et out
        return gsmERR;

    tcsetattr(fd_serial, TCSAFLUSH, &config); //flush
    return gsmOK;
}

static gsmr_t
configure_uart(uint32_t baudrate, const char* device_name) {
    if(fd_serial == -1){
        fd_serial = open(device_name, O_RDWR | O_NOCTTY); //Ouverture du device
        if(fd_serial < 0) //Si erreur d'ouverture
            return gsmERR;
    }

    gsmr_t ret_baudrate = set_baudrate(baudrate);
    if(ret_baudrate != gsmOK)
        return ret_baudrate;

    if(!initialized)    //create thread for reception
    {
        if(pthread_create(&rx_thread, NULL, &uart_thread_func, NULL) != 0) {
            return gsmERR;
        }
    }
    return gsmOK;
}

/**
 * \brief            UART thread
 */
static void*
uart_thread_func(void* param) {
    ssize_t bytes_read;
    uint8_t data_buffer[0x1000];             /*!< Received data array */

    while (1) {
        /*
         * Try to read data from uart port
         * and send it to upper layer for processing
         */
        do {
            bytes_read = read(fd_serial, data_buffer, sizeof(data_buffer));
            if(bytes_read < 0){  //there was an error
                sleep(1);
                continue;
            }
            /* Send received data to input processing module */
#if GSM_CFG_INPUT_USE_PROCESS
                gsm_input_process(data_buffer, (size_t)bytes_read);
#else /* GSM_CFG_INPUT_USE_PROCESS */
                gsm_input(data_buffer, (size_t)bytes_read);
#endif /* !GSM_CFG_INPUT_USE_PROCESS */
        } while (bytes_read == sizeof(data_buffer));

        /* Implement delay to allow other tasks processing */
        sleep(1);
    }
}

/**
 * \brief           Callback function called from initialization process
 *
 * \note            This function may be called multiple times if AT baudrate is changed from application.
 *                  It is important that every configuration except AT baudrate is configured only once!
 *
 * \note            This function may be called from different threads in GSM stack when using OS.
 *                  When \ref GSM_CFG_INPUT_USE_PROCESS is set to 1, this function may be called from user UART thread.
 *
 * \param[in,out]   ll: Pointer to \ref gsm_ll_t structure to fill data for communication functions
 * \return          gsmOK on success, member of \ref gsmr_t enumeration otherwise
 */
gsmr_t
gsm_ll_init(gsm_ll_t* ll) {
#if !GSM_CFG_MEM_CUSTOM
    static uint8_t memory[GSM_MEM_SIZE];
    gsm_mem_region_t mem_regions[] = {
        { memory, sizeof(memory) }
    };

    if (!initialized) {
        gsm_mem_assignmemory(mem_regions, GSM_ARRAYSIZE(mem_regions));  /* Assign memory for allocations */
    }
#endif /* !GSM_CFG_MEM_CUSTOM */

    /* Step 2: Set AT port send function to use when we have data to transmit */
    if (!initialized) {
        ll->send_fn = send_data;                /* Set callback function to send data */
    }

    /* Step 3: Configure AT port to be able to send/receive data to/from GSM device */
    if( configure_uart(ll->uart.baudrate, ll->uart.device_name) != gsmOK )          /* Initialize UART for communication */
        return gsmERR;
    initialized = 1;
    return gsmOK;
}

/**
 * \brief           Callback function to de-init low-level communication part
 * \param[in,out]   ll: Pointer to \ref gsm_ll_t structure to fill data for communication functions
 * \return          \ref gsmOK on success, member of \ref gsmr_t enumeration otherwise
 */
gsmr_t
gsm_ll_deinit(gsm_ll_t* ll) {
    pthread_join(rx_thread, NULL);
    initialized = 0;                            /* Clear initialized flag */
    return gsmOK;
}
