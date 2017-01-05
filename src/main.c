/*
 * Copyright 2017 Dius Computing Pty Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Johny Mattsson <jmattsson@dius.com.au>
 */

// Inspiration drawn from the old Python script "cp210x-program" @
//   http://cp210x-program.sourceforge.net/
// Alas I could not get it to work with the CP2105 chip I needed to reprogram,
// hence this rewrite.

#include <libusb-1.0/libusb.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#define CP210X_CFG 0xff
#define TIMEOUT_MS 500
#define MAX_SERIAL 128


#define MODEL(n) (1 << n)
#define MODELS_ALL (0xffffffff)
#define MODELS_123 (MODEL(1) | MODEL(2) | MODEL(3))

#define ITEM_VID   0x3701
#define ITEM_PID   0x3702
#define ITEM_NAME  0x3703
#define ITEM_SERI  0x3704
#define ITEM_MODEL 0x370b
#define ITEM_FLUSH 0x370d
#define ITEM_MODE  0x3711

typedef struct
{
  uint16_t vid;
  uint16_t pid;
} vid_pid_t;

static const vid_pid_t known[] =
{
  { 0x10C4, 0xEA60 },
  { 0x10C4, 0xEA70 }
};

typedef struct
{
  uint16_t value;
  uint16_t value_offs;
  uint32_t len;
  enum { CFG_INT8, CFG_INT16, CFG_STR } type;
  uint32_t on_model;
  bool write_only;
  const char *fmtstr;
} config_param_t;

#define WO true
#define RW false

static const config_param_t cfg_items[] =
{
  // Note: model entry must be first
  { ITEM_MODEL, '0', 1, CFG_INT8, MODELS_ALL, RW, "Model: CP210%c\n" },

  // Aaarrgh - attempts to READ these through the vendor interface
  // got interpreted as a WRITE! Stupid 0000:0000 VID:PID *mutter*grumble*
  { ITEM_VID,  0, 2, CFG_INT16, MODELS_123, WO, "Vendor ID: %04x\n" },
  { ITEM_PID,  0, 2, CFG_INT16, MODELS_123, WO, "Product ID: %04x\n" },
  { ITEM_NAME, 0, 255, CFG_STR, MODELS_123, WO, "Name: %s\n" },
  { ITEM_SERI, 0, MAX_SERIAL, CFG_STR, MODELS_123, WO, "Serial: %s\n" },

  { ITEM_FLUSH, 0, 1, CFG_INT8,  MODEL(5), RW, "Flush buffers: %x\n" },
  { ITEM_MODE,  0, 2, CFG_INT16, MODEL(5), RW, "SCI/ECI mode: %04hx\n" },
};

static void print_cp210x_cfg (libusb_device_handle *cp210x)
{
  uint8_t model = 0;
  for (unsigned i = 0; i < (sizeof (cfg_items)/sizeof (cfg_items[0])); ++i)
  {
    const config_param_t *item = &cfg_items[i];
    char buffer[256] = { 0, };
    if (item->write_only || !(i == 0 || (item->on_model & MODEL(model))))
      continue;
    int ret = libusb_control_transfer (cp210x,
      LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
      CP210X_CFG, item->value, 0, (uint8_t *)buffer, item->len, TIMEOUT_MS);
    if (ret < 0)
      fprintf (stderr, "error: failed to read cfg item %04x: %s\n",
        item->value, libusb_strerror (ret));
     else
     {
       unsigned uret = (unsigned)ret;
       buffer[uret < sizeof (buffer) ? uret : sizeof (buffer) -1] = 0;
       switch (item->type)
       {
         case CFG_INT8:
           printf (item->fmtstr, (int)buffer[0] + item->value_offs);
           if (i == 0)
             model = (uint8_t)buffer[0];
           break;
         case CFG_INT16:
           printf (item->fmtstr, (int)buffer[0] << 8 | buffer[1]);
           break;
         case CFG_STR:
           printf (item->fmtstr, buffer);
           break;
       }
     }
  }
}

static bool cp210x_set_cfg (libusb_device_handle *cp210x, uint16_t itemno, uint16_t index, uint8_t *data, uint16_t len)
{
  int ret = libusb_control_transfer (cp210x,
    LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
    CP210X_CFG, itemno, index, data, len, TIMEOUT_MS);
    if (ret < 0)
      fprintf (stderr, "error: failed to set cfg item %04x: %s\n",
        itemno, libusb_strerror (ret));
  return ret >= 0;
}

static bool cp210x_set_vid (libusb_device_handle *cp210x, uint16_t vid)
{
  return cp210x_set_cfg (cp210x, ITEM_VID, vid, NULL, 0);
}

static bool cp210x_set_pid (libusb_device_handle *cp210x, uint16_t pid)
{
  return cp210x_set_cfg (cp210x, ITEM_PID, pid, NULL, 0);
}

static bool cp210x_set_flush (libusb_device_handle *cp210x, uint8_t bitmap)
{
  return cp210x_set_cfg (cp210x, ITEM_FLUSH, 0, &bitmap, 1);
}

static bool cp210x_set_mode (libusb_device_handle *cp210x, uint16_t mode)
{
  return cp210x_set_cfg (cp210x, ITEM_MODE, 0, (uint8_t *)&mode, 2);
}

static bool encode_descriptor_string (uint8_t out[256], const char *str)
{
  int len = strlen (str);
  if (len > 126)
  {
    fprintf (stderr, "error: descriptor string is too long\n");
    return false;
  }
  out[0] = len * 2 + 2;
  out[1] = 0x03;
  // poor-man's utf-16-le encoding
  for (int i = 0, o = 2; i < len; ++i, o += 2)
  {
    if (str[i] & 0x80)
    {
      fprintf (stderr, "error: only ASCII descriptor strings supported\n");
      return false;
    }
    out[o] = str[i];
    out[o+1] = 0;
  }
  return true;
}

static bool cp210x_set_name (libusb_device_handle *cp210x, const char *name)
{
  uint8_t buffer[256];
  if (!encode_descriptor_string (buffer, name))
    return false;
  return cp210x_set_cfg (cp210x, ITEM_NAME, 0, buffer, buffer[0]);
}

static bool cp210x_set_serial (libusb_device_handle *cp210x, const char *serial)
{
  uint8_t buffer[256];
  if (!encode_descriptor_string (buffer, serial))
    return false;
  if (buffer[0] > MAX_SERIAL)
  {
    fprintf (stderr, "error: serial string too long\n");
    return false;
  }
  return cp210x_set_cfg (cp210x, ITEM_SERI, 0, buffer, buffer[0]);
}


static bool cp210x_reset (libusb_device_handle *cp210x)
{
  return libusb_reset_device (cp210x) == 0;
}


static bool recognised_cp210x_dev (uint16_t vid, uint16_t pid)
{
  for (unsigned i = 0; i < (sizeof (known)/sizeof (known[0])); ++i)
  {
    if (known[i].vid == vid && known[i].pid == pid)
      return true;
  }
  return false;
}

void syntax (void)
{
  fprintf (stderr,
"Syntax:\n"
"cp210x-cfg [-h ] |\n"
"           [-m vid:pid] [-d bus:dev]\n"
"           [ -l | [-V vid] [-P pid] [-F flush] [-M mode] [-N name] [-S serial]]\n" 
"\n"
"  -h            This help\n"
"  -m vid:pid    Find and use first device with vid:pid\n"
"  -d bus:dev    Find and use device at bus:dev\n"
"  -l            List all CP210x devices connected\n"
"  -V vid        Program the given Vendor ID\n"
"  -P pid        Program the given Product ID\n"
"  -F flush      Program the given buffer flush bitmap (CP2105 only)\n"
"  -M mode       Program the given SCI/ECI mode (CP2105 only)\n"
"  -N name       Program the given product name string\n"
"  -S serial     Program the given serial string\n"
"\n"
"Unless the -d option is used, the first found CP210x device is used.\n"
"If no programming options are used, the current values are printed.\n"
"\n"
  ); 
}

#define usb_err_out(msg,err,ec) \
  do { \
    fprintf (stderr, "error: " msg ": %s\n", libusb_strerror (err)); \
    exitcode = ec; \
    goto out; \
  } while (0)

int main (int argc, char *argv[])
{
  bool want_list = false;
  bool want_bus_dev = false;
  int want_bus = -1;
  int want_dev = -1;
  bool want_vid_pid = false;
  vid_pid_t want_vp = { 0, 0 };

  bool set_vid = false;
  uint16_t new_vid = 0x10c4;
  bool set_pid = false;
  uint16_t new_pid = 0xea70;
  bool set_flush = false;
  uint8_t new_flush = 0x33;
  bool set_mode = false;
  uint16_t new_mode = 0x0000;
  bool set_name = false;
  const char *new_name = 0;
  bool set_serial = false;
  const char *new_serial = 0;

  int exitcode = 0;

  int opt;
  while ((opt = getopt (argc, argv, "rhld:m:V:P:F:M:N:S:")) != -1)
  {
    switch (opt)
    {
      case 'h': syntax (); return 0;
      case 'l': want_list = true; break;
      case 'd':
      {
        want_bus_dev = true;
        char *p = NULL;
        want_bus = strtol (optarg, &p, 10);
        if (*p != '.')
        {
          fprintf (stderr, "error: bad format to -d option, expected n.n\n");
          return 10;
        }
        want_dev = strtol (++p, NULL, 0);
        break;
      }
      case 'm':
      {
        want_vid_pid = true;
        char *p = NULL;
        want_vp.vid = (uint16_t)strtol (optarg, &p, 16);
        if (*p != ':')
        {
          fprintf (stderr, "error: bad format to -m option, expected xxxx:xxxx\n");
          return 11;
        }
        want_vp.pid = (uint16_t)strtol (++p, NULL, 16);
        break;
      }
      case 'V': set_vid    = true; new_vid    = strtol (optarg, 0, 16); break;
      case 'P': set_pid    = true; new_pid    = strtol (optarg, 0, 16); break;
      case 'F': set_flush  = true; new_flush  = strtol (optarg, 0, 16); break;
      case 'M': set_mode   = true; new_mode   = strtol (optarg, 0, 16); break;
      case 'N': set_name   = true; new_name   = optarg; break;
      case 'S': set_serial = true; new_serial = optarg; break;
      default:
        fprintf (stderr, "error: unknown option '%c'\n", opt);
        return 12;
    }
  }

  libusb_context *usb = 0;
  libusb_device_handle *cp210x = 0;
  struct libusb_device_descriptor desc;

  int ret = libusb_init (&usb);
  if (ret != 0)
    usb_err_out ("Failed to open libusb", ret, 1);

  libusb_device **devices;
  ssize_t n = libusb_get_device_list (usb, &devices);
  if (n < 0)
    usb_err_out ("Failed to list USB devices", ret, 2);
  for (ssize_t i = 0; i < n; ++i)
  {
    memset (&desc, 0, sizeof (desc));
    libusb_get_device_descriptor (devices[i], &desc);
    uint16_t vid = desc.idVendor;
    uint16_t pid = desc.idProduct;
    int bus = libusb_get_bus_number (devices[i]);
    int dev = libusb_get_device_address (devices[i]);

    if ((want_vid_pid && vid == want_vp.vid && pid == want_vp.pid) ||
        (want_bus_dev && bus == want_bus && want_dev == dev) ||
        (!want_vid_pid && !want_bus_dev &&
         recognised_cp210x_dev (desc.idVendor, desc.idProduct)))
    {
      ret = libusb_open (devices[i], &cp210x);
      if (ret != 0)
        usb_err_out ("Failed to open device", ret, 4);

      char buffer[256] = { 0, };
      ret = libusb_get_string_descriptor_ascii (
        cp210x, desc.iProduct, (unsigned char *)buffer, sizeof (buffer));
      if (ret < 0)
        usb_err_out ("Failed to read descriptor", ret, 5);

      printf ("ID %04x:%04x @ bus %03d, dev %03d: %s\n",
        vid, pid, bus, dev, buffer);
      if (want_list)
      {
        libusb_close (cp210x);
        cp210x = 0;
        continue;
      }
      else
        break;
    }
  }
  libusb_free_device_list (devices, 1);

  if (want_list)
    goto out; // done now

  if (!cp210x)
  {
    fprintf (stderr, "error: No device found.\n");
    exitcode = 3;
    goto out;
  }

  libusb_set_auto_detach_kernel_driver (cp210x, 1);

  if (set_vid)
    exitcode += cp210x_set_vid (cp210x, new_vid) ? 0 : 16;
  if (set_pid)
    exitcode += cp210x_set_pid (cp210x, new_pid) ? 0 : 32;
  if (set_flush)
    exitcode += cp210x_set_flush (cp210x, new_flush) ? 0 : 64;
  if (set_mode)
    exitcode += cp210x_set_mode (cp210x, new_mode) ? 0 : 128;
  if (set_name)
    exitcode += cp210x_set_name (cp210x, new_name) ? 0 : 256;
  if (set_serial)
    exitcode += cp210x_set_serial (cp210x, new_serial) ? 0 : 512;

  if (set_vid || set_pid || set_flush || set_mode || set_name || set_serial)
    cp210x_reset (cp210x);

  print_cp210x_cfg (cp210x);

out:
  if (cp210x)
    libusb_close (cp210x);
  if (usb)
    libusb_exit (usb);
  return exitcode;
}
