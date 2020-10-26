TARGET=sample
EPS=$(shell ls *.eps)
TEX=platex
DVIPDF=dvipdfmx
ARCHIVE=mstf101
FILES=mstf.sty makefile $(EPS)

.SUFFIXES: .tex .dvi

all:$(TARGET).dvi

$(TARGET).dvi: $(TARGET).tex $(EPS)

.tex.dvi:
	$(TEX) $<
	$(TEX) $<

tar:
	tar cvzf $(ARCHIVE).tar.gz $(TARGET).* $(FILES) 

lzh:
	lha a $(ARCHIVE).lzh $(TARGET).* $(FILES)


pdf: $(TARGET).dvi
	$(DVIPDF) $(TARGET)

clean:
	rm -f $(TARGET).dvi $(TARGET).aux $(TARGET).log 
	rm -f $(TARGET).pdf $(TARGET).tar.gz 
	rm -f *.pbm *.bmc
