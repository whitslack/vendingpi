ZLIB_CPPFLAGS := $(shell $(PKG_CONFIG) --cflags zlib)
ZLIB_LDLIBS := $(shell $(PKG_CONFIG) --libs zlib)
GNUTLS_CPPFLAGS := $(shell $(PKG_CONFIG) --cflags gnutls)
GNUTLS_LDLIBS := $(shell $(PKG_CONFIG) --libs gnutls)

ifeq ($(VERSION),)
VERSION := $(shell git describe --always --dirty)
endif
CPPFLAGS += -DVERSION='"$(VERSION)"'

ifndef RPI_REV
$(error Must specify RPI_REV)
endif
CPPFLAGS += -DRPI_REV='$(RPI_REV)'

SOURCES += $(wildcard *.cpp common/*.cpp libsatoshi/*.cpp)

CPPFLAGS += -I. -DHTTP_GZIP -DHTTP_TLS


VENDING_OBJECTS := $(addprefix $(OBJDIR)/,bitcoin.o bitcoin_price.o gpio.o vending.o $(addprefix common/,connect.o dns.o ecp.o eventfd.o fd.o fp.o gzip.o http.o io.o json.o linux_aio.o log.o mpn.o murmur3.o regex.o ripemd.o sha.o signal.o socket.o termios.o timer.o tls.o webclient.o) $(addprefix libsatoshi/,base58check.o blockchain.o bloom.o node.o satoshi.o script.o types.o))
VENDING := $(BINDIR)/vending$(EXEC_SUFFIX)
ALL += $(VENDING)

$(VENDING) : CPPFLAGS += -pthread $(ZLIB_CPPFLAGS) $(GNUTLS_CPPFLAGS)
$(VENDING) : LDLIBS += -lrt -lgmp -laio $(ZLIB_LDLIBS) $(GNUTLS_LDLIBS)
$(VENDING) :: $(VENDING_OBJECTS)
	$(DO_LINK.cpp)
