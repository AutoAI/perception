CC	= cc 
CFLAGS	= -O
LDFLAGS	= -lm
LINT	= lint
PRINT	= ato2ps -Ple
INCS 	=\
	decl.h\
	defs.h\
	extern.h\
	dc.h\
	edge.h

SRC 	=\
	dc.c\
	edge.c\
	error.c\
	i_o.c\
	main.c\
	memory.c\
	sort.c


OBJS 	=\
	dc.o\
	edge.o\
	error.o\
	i_o.o\
	main.o\
	memory.o\
	sort.o

TEXT	= Makefile README $(INCS) $(SRC) 

dct	: $(OBJS)
	$(CC) -o $@ $(CFLAGS) $(OBJS) $(LDFLAGS)

lint	: $(SRC) $(INCS)
	$(LINT) $(SRC)

print	:
	$(PRINT) $(TEXT)

tar	:
	tar cvf dct.tar $(TEXT)

DINCS	=\
	defs.h\
	decl.h

$(OBJS) 	: $(DINCS)

edge.o	: edge.h 
dc.o	: dc.h edge.h
