include ../Makefile.in

BINDIR = ..

INCLUDES =  -I../multilevelLib -I../metisLib -I/Users/ivanrodriguezmurillo/Documents/Source/graclus1.2/metisLib  -I/Users/ivanrodriguezmurillo/Documents/Source/graclus1.2/multilevelLib -I/usr/local/lib -I/opt/local/include
CFLAGS = $(COPTIONS) $(OPTFLAGS) $(INCLUDES)

LIBS = -lmultilevel -lmetis -lm

LD = $(CC) $(LDOPTIONS)  -L. -L.. -L/usr/local/lib -L/opt/local/lib
METISLIB = ../libmetis.a ../libmultilevel.a

MLKKMOBJS = graclus.o io.o


.c.o:
	$(CC) $(CFLAGS) -c $*.c

default: $(BINDIR)/graclus$(VERNUM)


$(BINDIR)/graclus$(VERNUM): $(MLKKMOBJS) $(METISLIB)
	$(LD)  -o $@ $(CFLAGS) $(MLKKMOBJS) $(LIBS)
	chmod 755 $@



clean:
	rm -f *.o

realclean:
	rm -f *.o ;\
	rm -f $(BINDIR)/graclus$(VERNUM) 
