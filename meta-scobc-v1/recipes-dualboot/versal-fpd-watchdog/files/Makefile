APP  := versal-fpd-watchdog-daemon
SRCS := versal-fpd-watchdog-daemon.c
OBJS := $(SRCS:.c=.o)
DEPS := $(SRCS:.c=.d)

.PHONY: all clean install

all: $(APP)

$(APP): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.c
	$(CC) -MMD -MP -c $< -o $@ $(CFLAGS) $(CPPFLAGS)

install: $(APP)
	install -d $(DESTDIR)$(BINDIR)
	install -m 0755 $(APP) $(DESTDIR)$(BINDIR)

clean:
	rm -f $(APP) $(OBJS) $(DEPS)

-include $(DEPS)
