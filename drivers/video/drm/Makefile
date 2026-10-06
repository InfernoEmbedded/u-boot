# SPDX-License-Identifier: GPL-2.0-or-later

obj-y += sunxi_drm_drv.o
obj-y += sunxi_drm_crtc.o sunxi_drm_connector.o sunxi_drm_bridge.o sunxi_drm_helper_funcs.o \
	drm_fourcc.o drm_modes.o drm_plane.o drm_edid.o drm_displayid.o drm_framebuffer.o load_file.o

ccflags-y += -I$(srctree)/$(obj)/sunxi_device/hardware/
ccflags-y += -I$(srctree)/$(obj)/sunxi_device/hardware/lowlevel_de/
ccflags-y += -I$(srctree)/$(obj)/sunxi_device/hardware/lowlevel_lcd/
ccflags-y += -I$(srctree)/$(obj)/sunxi_device/hardware/lowlevel_tcon/
ccflags-y += -I$(srctree)/$(obj)/sunxi_device/hardware/lowlevel_hdmi20/
ccflags-y += -I$(srctree)/$(obj)/

#tcon
include $(srctree)/$(obj)/sunxi_device/hardware/lowlevel_tcon/Makefile
obj-y += $(tcon_obj)

#de
include $(srctree)/$(obj)/sunxi_device/hardware/lowlevel_de/Makefile
obj-$(CONFIG_AW_DRM_DE) += $(de_obj)

obj-$(CONFIG_AW_DRM_TCON) += sunxi_device/sunxi_tcon.o
obj-$(CONFIG_AW_DRM_TCON) += sunxi_device/hardware/lowlevel_lcd/tcon_lcd.o

obj-$(CONFIG_AW_DRM_TCON_TOP) += sunxi_device/sunxi_tcon_top.o

obj-$(CONFIG_AW_DRM_HDMI_TX) += sunxi_drm_hdmi.o
obj-$(CONFIG_AW_DRM_HDMI_TX) += sunxi_device/sunxi_hdmi.o
include $(srctree)/$(obj)/sunxi_device/hardware/lowlevel_hdmi20/Makefile
obj-$(CONFIG_AW_DRM_HDMI_TX) += $(obj_low_hdmi)
