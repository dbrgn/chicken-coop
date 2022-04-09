################################################################################
#
# nginx-rtmp-module
#
################################################################################

NGINX_RTMP_MODULE_VERSION = 1.2.2
NGINX_RTMP_MODULE_SITE = $(call github,arut,nginx-rtmp-module,v$(NGINX_RTMP_MODULE_VERSION))
NGINX_RTMP_MODULE_LICENSE = BSD-2-Clause
NGINX_RTMP_MODULE_LICENSE_FILES = LICENSE
NGINX_RTMP_MODULE_DEPENDENCIES = openssl

NGINX_DEPENDENCIES += nginx-rtmp-module
NGINX_CONF_OPTS += --add-module=$(NGINX_RTMP_MODULE_DIR)

$(eval $(generic-package))
