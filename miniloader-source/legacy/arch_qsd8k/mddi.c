/*
 * Copyright (c) 2008, Google Inc.
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

#include <boot/boot.h>
#include <boot/board.h>
#include <qsd8k/mddi.h>

#define MSM_MDDI_BASE g_mddi_base
static unsigned int g_mddi_base = MSM_MDDI_PHY_BASE;
unsigned fb_width = 0;
unsigned fb_height = 0;

static unsigned short *FB;
static mddi_llentry *mlist;

void wr32(void *_dst, unsigned n)
{
    unsigned char *src = (unsigned char*) &n;
    unsigned char *dst = _dst;

    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
};

void mddi_set_caps(mddi_client_caps *c)
{
        /* Hardcoding the capability values */
    c->length = 74;
    c->type = 66;
    c->client_id = 0;
    c->protocol_ver = 1;
    c->min_protocol_ver = 1;
    c->data_rate_cap = 400;
    c->interface_type_cap = 0;
    c->num_alt_displays = 1;
    c->postcal_data_rate = 400;
    c->bitmap_width = 480;
    c->bitmap_height = 800;
    c->display_window_width = 480;
    c->display_window_height = 800;
    c->cmap_size = 0;
    c->cmap_rgb_width = 0;
    c->rgb_cap = 34592;
    c->mono_cap = 0;
    c->reserved1 = 0;
    c->ycbcr_cap = 0;
    c->bayer_cap = 0;
    c->alpha_cursor_planes = 0;
    c->client_feature_cap = 4489216;
    c->max_video_frame_rate_cap = 60;
    c->min_video_frame_rate_cap = 0;
    c->min_sub_frame_rate = 0;
    c->audio_buf_depth = 0;
    c->audio_channel_cap = 0;
    c->audio_sampe_rate_rap = 0;
    c->audio_sample_res = 0;
    c->mic_audio_sample_res = 0;
    c->mic_sample_rate_cap = 0;
    c->keyboard_data_fmt = 0;
    c->pointing_device_data_fmt = 0;
    c->content_protection_type = 0;
    c->manufacturer_name = 53859;
    c->product_code = 34594;
    c->reserved3 = 0;
    c->serial_no = 1;
    c->week_of_manufacture = 0;
    c->year_of_manufacture = 0;
    c->crc = 53536;
}

void printcaps(mddi_client_caps *c)
{
    if((c->length != 0x4a) || (c->type != 0x42)) {
        dprintf("bad caps header received, so hardcoding\n");
        memset(c, 0, sizeof(*c));
        mddi_set_caps(c);
    }

    dprintf("mddi: bm: %d,%d win %d,%d rgb %x\n",
            c->bitmap_width, c->bitmap_height,
            c->display_window_width, c->display_window_height,
            c->rgb_cap);
    dprintf("mddi: vend %x prod %x\n",
            c->manufacturer_name, c->product_code);

    fb_width = c->bitmap_width;
    fb_height = c->bitmap_height;

    panel_init(c);
}

mddi_llentry *mlist_remote_write = 0;

void mddi_remote_write(unsigned val, unsigned reg)
{
    mddi_llentry *ll;
    mddi_register_access *ra;
    unsigned s;

    if(mlist_remote_write == 0) {
        mlist_remote_write = alloc(sizeof(mddi_llentry));
    }

    ll = mlist_remote_write;

    ra = &(ll->u.r);
    ra->length = 14 + 4;
    ra->type = TYPE_REGISTER_ACCESS;
    ra->client_id = 0;
    ra->rw_info = MDDI_WRITE | 1;
    ra->crc = 0;

    wr32(&ra->reg_addr, reg);
    wr32(&ra->reg_data, val);

    ll->flags = 1;
    ll->header_count = 14;
    ll->data_count = 4;
    wr32(&ll->data, (unsigned) &ra->reg_data);
    wr32(&ll->next, 0);
    ll->reserved = 0;

    writel((unsigned) ll, MDDI_PRI_PTR);

    s = readl(MDDI_STAT);
    while((s & 0x20) == 0){
        s = readl(MDDI_STAT);
    }
}

void mddi_start_update(void)
{
    writel((unsigned) mlist, MDDI_PRI_PTR);
}

int mddi_update_done(void)
{
    return !!(readl(MDDI_STAT) & MDDI_STAT_PRI_LINK_LIST_DONE);
}

void mddi_do_cmd(unsigned cmd)
{
    writel(cmd, MDDI_CMD);

    while (!(readl(MDDI_INT) & MDDI_INT_NO_REQ_PKTS_PENDING)) ;
}

unsigned char *rev_pkt_buf;

void mddi_get_caps(void)
{
    unsigned timeout = 100000;
    unsigned n;

    memset(rev_pkt_buf, 0xee, 256);

//    writel(CMD_HIBERNATE, MDDI_CMD);
//    writel(CMD_LINK_ACTIVE, MDDI_CMD);

    writel(256, MDDI_REV_SIZE);
    writel((unsigned) rev_pkt_buf, MDDI_REV_PTR);
    mddi_do_cmd(CMD_FORCE_NEW_REV_PTR);

        /* sometimes this will fail -- do it three times for luck... */
    mddi_do_cmd(CMD_RTD_MEASURE);
    mdelay(1);

    mddi_do_cmd(CMD_RTD_MEASURE);
    mdelay(1);

    mddi_do_cmd(CMD_RTD_MEASURE);
    mdelay(1);

    mddi_do_cmd(CMD_GET_CLIENT_CAP);

    do {
        n = readl(MDDI_INT);
    } while(!(n & MDDI_INT_REV_DATA_AVAIL) && (--timeout));

    if(timeout == 0) dprintf("timeout\n");
    printcaps((mddi_client_caps*) rev_pkt_buf);
}


#if 1
void mddi_setbase(unsigned int base)
{
    g_mddi_base = base;
}

void mddi_init(void)
{
    unsigned n;
    int i = 0;
//    dprintf("mddi_init()\n");

    rev_pkt_buf = alloc(256);
    
    writel((unsigned) rev_pkt_buf, MDDI_REV_PTR);
    writel(256, MDDI_REV_SIZE);
    writel(256, MDDI_REV_ENCAP_SZ);

    mddi_do_cmd(CMD_FORCE_NEW_REV_PTR);
    mddi_do_cmd(CMD_LINK_ACTIVE);

    do {
        if(i++ > 0x100) break;
        n = readl(MDDI_STAT);
    } while(!(n & MDDI_STAT_LINK_ACTIVE));

        /* v > 8?  v > 8 && < 0x19 ? */
    writel(2, MDDI_TEST);

//    writel(CMD_PERIODIC_REV_ENC | 0, MDDI_CMD); /* disable */
    fb_width = 480;
    fb_height = 854;

    dprintf("panel is %d x %d\n", fb_width, fb_height);

//    FB = alloc(2 * fb_width * fb_height);
    FB = 0x2b00000;
    mlist = alloc(sizeof(mddi_llentry) * (fb_height / 8));

//    dprintf("FB @ %x  mlist @ %x\n", (unsigned) FB, (unsigned) mlist);

    for(n = 0; n < (fb_height / 8); n++) {
        unsigned y = n * 8;
        unsigned pixels = fb_width * 8;
        mddi_video_stream *vs = &(mlist[n].u.v);

        vs->length = sizeof(mddi_video_stream) - 2 + (pixels * 2);
        vs->type = TYPE_VIDEO_STREAM;
        vs->client_id = 0;
        vs->format = 0x5565; // FORMAT_16BPP;
        vs->pixattr = PIXATTR_BOTH_EYES | PIXATTR_TO_ALL;

        vs->left = 0;
        vs->right = fb_width - 1;
        vs->top = y;
        vs->bottom = y + 7;

        vs->start_x = 0;
        vs->start_y = y;
 
        vs->pixels = pixels;
        vs->crc = 0;
        vs->reserved = 0;

        mlist[n].header_count = sizeof(mddi_video_stream) - 2;
        mlist[n].data_count = pixels * 2;
        mlist[n].reserved = 0;
        wr32(&mlist[n].data, ((unsigned) FB) + (y * fb_width * 2));

        mlist[n].flags = 0;
        wr32(&mlist[n].next, (unsigned) (mlist + n + 1));
    }

    mlist[n-1].flags = 1;
    wr32(&mlist[n-1].next, 0);

    writel(CMD_HIBERNATE, MDDI_CMD);
    writel(CMD_LINK_ACTIVE, MDDI_CMD);

    for(n = 0; n < (fb_width * fb_height); n++) FB[n] = 0;//0x88;

    mddi_start_update();
}
#else
void mddi_init(void)
{
    unsigned n;

//    dprintf("mddi_init()\n");

    rev_pkt_buf = alloc(256);
    
    mddi_do_cmd(CMD_RESET);

        /* disable periodic rev encap */
    mddi_do_cmd(CMD_PERIODIC_REV_ENC | 0);

    writel(0x0001, MDDI_VERSION);
    writel(0x3C00, MDDI_BPS);
    writel(0x0003, MDDI_SPM);

    writel(0x0005, MDDI_TA1_LEN);
    writel(0x001a, MDDI_TA2_LEN);

    writel(0x0096, MDDI_DRIVE_HI);
    writel(0x0050, MDDI_DRIVE_LO);
    writel(0x003C, MDDI_DISP_WAKE);
    writel(0x0004, MDDI_REV_RATE_DIV);

        /* needs to settle for 5uS */
    if (readl(MDDI_PAD_CTL) == 0) {
        writel(0x08000, MDDI_PAD_CTL);
        udelay(5);
    }

    writel(0xA850F, MDDI_PAD_CTL);
    writel(0x60006, MDDI_DRIVER_START_CNT);

    writel((unsigned) rev_pkt_buf, MDDI_REV_PTR);
    writel(256, MDDI_REV_SIZE);
    writel(256, MDDI_REV_ENCAP_SZ);

    mddi_do_cmd(CMD_FORCE_NEW_REV_PTR);

    /* disable hibernate */
    mddi_do_cmd(CMD_HIBERNATE | 0);

    panel_backlight(0);

    panel_poweron();

    mddi_do_cmd(CMD_LINK_ACTIVE);

    do {
        n = readl(MDDI_STAT);
    } while(!(n & MDDI_STAT_LINK_ACTIVE));

        /* v > 8?  v > 8 && < 0x19 ? */
    writel(2, MDDI_TEST);

//    writel(CMD_PERIODIC_REV_ENC | 0, MDDI_CMD); /* disable */

    mddi_get_caps();

#if 0
    writel(0x5666, MDDI_MDP_VID_FMT_DES);
    writel(0x00C3, MDDI_MDP_VID_PIX_ATTR);
    writel(0x0000, MDDI_MDP_CLIENTID);
#endif

    dprintf("panel is %d x %d\n", fb_width, fb_height);

    FB = alloc(2 * fb_width * fb_height);
    mlist = alloc(sizeof(mddi_llentry) * (fb_height / 8));

//    dprintf("FB @ %x  mlist @ %x\n", (unsigned) FB, (unsigned) mlist);

    for(n = 0; n < (fb_height / 8); n++) {
        unsigned y = n * 8;
        unsigned pixels = fb_width * 8;
        mddi_video_stream *vs = &(mlist[n].u.v);

        vs->length = sizeof(mddi_video_stream) - 2 + (pixels * 2);
        vs->type = TYPE_VIDEO_STREAM;
        vs->client_id = 0;
        vs->format = 0x5565; // FORMAT_16BPP;
        vs->pixattr = PIXATTR_BOTH_EYES | PIXATTR_TO_ALL;

        vs->left = 0;
        vs->right = fb_width - 1;
        vs->top = y;
        vs->bottom = y + 7;

        vs->start_x = 0;
        vs->start_y = y;
 
        vs->pixels = pixels;
        vs->crc = 0;
        vs->reserved = 0;

        mlist[n].header_count = sizeof(mddi_video_stream) - 2;
        mlist[n].data_count = pixels * 2;
        mlist[n].reserved = 0;
        wr32(&mlist[n].data, ((unsigned) FB) + (y * fb_width * 2));

        mlist[n].flags = 0;
        wr32(&mlist[n].next, (unsigned) (mlist + n + 1));
    }

    mlist[n-1].flags = 1;
    wr32(&mlist[n-1].next, 0);

    writel(CMD_HIBERNATE, MDDI_CMD);
    writel(CMD_LINK_ACTIVE, MDDI_CMD);

    for(n = 0; n < (fb_width * fb_height); n++) FB[n] = 0;

    mddi_start_update();

    panel_backlight(1);
}
#endif

void *mddi_framebuffer(void)
{
    return FB;
}


