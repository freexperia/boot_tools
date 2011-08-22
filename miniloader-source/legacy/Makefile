.PHONY : all
all:
	cd arch_qsd8k && $(MAKE)
	cd arch_armv6 && $(MAKE)
	cd libboot && $(MAKE)
	cd libc && $(MAKE)
	cd miniloader && $(MAKE)

.PHONY : clean
clean:
	cd arch_qsd8k && $(MAKE) clean
	cd arch_armv6 && $(MAKE) clean
	cd libboot && $(MAKE) clean
	cd libc && $(MAKE) clean
	cd miniloader && $(MAKE) clean
