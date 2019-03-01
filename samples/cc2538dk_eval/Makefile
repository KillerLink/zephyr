COM0?=/dev/ttyUSB1
COM1?=/dev/ttyUSB3
FBAUD?=460800
SBAUD?=460800
OPTS?=-e -w -v
BOARD?=cc2538dk
APP?=serial-basic

all:
	echo "nothing."

prepare:
	@if [ ! -d "$(APP)" ]; then\
		echo "project folder missing, exit...";\
		false;\
	else\
		 echo "project folder found, ok!";\
	fi
	@if [ ! -d "$(APP)/build" ]; then\
		echo "folder '$(APP)/build' missing, create...";\
		mkdir "$(APP)/build";\
	else\
		echo "folder '$(APP)/build' found, ok!";\
	fi
	@if [ ! -d "$(APP)/build/$(BOARD)" ]; then\
		echo "folder 'build/$(BOARD)' missing, create...";\
		mkdir "$(APP)/build/$(BOARD)";\
	else\
		echo "folder '$(APP)/build/$(BOARD)' found, ok!";\
	fi
	@if [ ! -f "$(APP)/build/$(BOARD)/Makefile" ]; then\
		echo "Makefile file missing, create...";\
		cmake -S "$(APP)/" -B "$(APP)/build/$(BOARD)/";\
	else\
		echo "Makefile already exists, ok!";\
	fi

clean:
	make clean -C $(APP)

build:
	make -C $(APP)/build/$(BOARD)/

flash0:
	./cc2538-bsl.py -p $(COM0) -b $(FBAUD) $(OPTS)  $(APP)/build/$(BOARD)/zephyr/zephyr.bin

flash1:
	./cc2538-bsl.py -p $(COM1) -b $(FBAUD) $(OPTS)  $(APP)/build/$(BOARD)/zephyr/zephyr.bin

serial0:
	stty -F $(COM0) $(SBAUD) -icrnl min 1 && cat $(COM0)

serial1:
	stty -F $(COM1) $(SBAUD) -icrnl min 1 && cat $(COM1)


