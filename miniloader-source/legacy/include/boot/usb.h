/*
 * Copyright (C) 2008 The Android Open Source Project
 * All rights reserved.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the 
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _USB_COMMON_DEFINES_H
#define _USB_COMMON_DEFINES_H

#define GET_STATUS           0
#define CLEAR_FEATURE        1
#define SET_FEATURE          3
#define SET_ADDRESS          5
#define GET_DESCRIPTOR       6
#define SET_DESCRIPTOR       7
#define GET_CONFIGURATION    8
#define SET_CONFIGURATION    9
#define GET_INTERFACE        10
#define SET_INTERFACE        11
#define SYNCH_FRAME          12

#define TYPE_DEVICE          1
#define TYPE_CONFIGURATION   2
#define TYPE_STRING          3
#define TYPE_INTERFACE       4
#define TYPE_ENDPOINT        5

#define DEVICE_READ          0x80
#define DEVICE_WRITE         0x00
#define INTERFACE_READ       0x81
#define INTERFACE_WRITE      0x01
#define ENDPOINT_READ        0x82
#define ENDPOINT_WRITE       0x02

#define PHY_TYPE_MASK      0xFF
#define PHY_MODEL_MASK     0xFF00
#define PHY_TYPE(x)     ((x) & PHY_TYPE_MASK)
#define PHY_MODEL(x)    ((x) & PHY_MODEL_MASK)

#define USB_PHY_MODEL_65NM 0x100
#define USB_PHY_MODEL_180NM   0x200
#define USB_PHY_UNDEFINED  0x00
#define USB_PHY_INTEGRATED 0x01
#define USB_PHY_EXTERNAL   0x02

/* control charger detection by ULPI or externally */
#define ULPI_EXTCHGCTRL_65NM  (1 << 2)
#define ULPI_EXTCHGCTRL_180NM (1 << 3)
#define ULPI_CHG_DETECT_REG     0x34//RIAZ: check if this is the right register

/* charger detection power on control */
#define ULPI_CHGDETON           (1 << 1)
 /* enable charger detection */
#define ULPI_CHGDETEN           (1 << 0)
#define ULPI_CHGTYPE_65NM  (1 << 3)
#define ULPI_CHGTYPE_180NM (1 << 4)

#define OTGSC_BSVIS            (1 << 19) /* R/W - BSV Interrupt Status */
#define B_SESSION_VALID        (1 << 11)

#define PORTSC_FPR             (1 << 6)  /* R/W - State normal => suspend */
#define PORTSC_SUSP            (1 << 7)  /* Read - Port in suspend state */
#define PORTSC_LS              (3 << 10) /* Read - Port's Line status */

typedef struct 
{
    unsigned char type;
    unsigned char request;
    unsigned short value;
    unsigned short index;
    unsigned short length;
} __attribute__ ((packed)) setup_packet;

enum charger_type {
   CHG_HOST_PC,
   CHG_WALL,
   CHG_UNDEFINED,
};


struct usb_request
{
    struct ept_queue_item *item;

    void *buf;
    unsigned length;
    
    void (*complete)(struct usb_request *req, unsigned actual, int status);
    void *context;
};

struct usb_request *usb_request_alloc();
struct usb_endpoint *usb_endpoint_alloc(unsigned num, unsigned in, unsigned maxpkt);
int usb_queue_req(struct usb_endpoint *ept, struct usb_request *req);

void usb_init(void);
void usb_shutdown(void);
void usb_poll(void);

/* called to indicate online/offline status */
void usb_status(unsigned online, unsigned highspeed);

#ifdef SURF8K
int is_usb_cable_connected(void);
void usb_charger_reset_8k(void);
void usb_stop_charging(unsigned);
unsigned is_usb_charging(void);
#endif

#endif
