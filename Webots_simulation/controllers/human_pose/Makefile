WEBOTS_HOME ?= $(shell webots --path)
INCLUDE = -I"$(WEBOTS_HOME)/include/controller/c"
LIBS = -L"$(WEBOTS_HOME)/lib/controller" -lController -lC
CFLAGS = -O2 -g

all:
	$(CC) human_pose.c -o human_pose $(INCLUDE) $(LIBS) $(CFLAGS)
