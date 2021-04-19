TARGET = libbCAPClient.so

CC = gcc
LIBS =  
INCS = -I../../Include/C/ -I../../Include/C/bCAPClient
DEFS = -D_USE_LINUX_API

BUILD_DIR = Build

SRCS = bcap_client.c \
  ../dn_common.c \
  ../dn_device.c \
  ../dn_socket.c \
  ../dn_tcp.c \
  ../dn_udp.c \
  ../dn_com.c \
  ../dn_thread.c \
  ../bcap_common.c

OBJS = $(addprefix $(BUILD_DIR)/,$(patsubst %.c,%.o,$(SRCS)))
DEPS = $(patsubst %.c,%.d,$(SRCS))

all: $(BUILD_DIR) $(TARGET)

-include $(DEPS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(OBJS)
	$(CC) -shared -fPIC -o $@ $(subst ../,,$^) $(LIBS) -lpthread

$(BUILD_DIR)/%.o: %.c
	echo $<; \
	mkdir -p $(dir $@); \
	$(CC) -c -MMD -MP -fPIC -o $(subst ../,,$@) $< $(INCS) $(DEFS)

install:
	mv $(TARGET) ../../Lib/

clean:
	rm -rf $(BUILD_DIR)
