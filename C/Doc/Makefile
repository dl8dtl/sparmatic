LATEX=	latex
PDFLATEX= pdflatex
DVIPS=	dvips
LATEX2RTF= latex2rtf

.SUFFIXES: .tex .dvi .ps .rtf .pdf

all: handbuch.pdf

printfile: handbuch-print.ps.bz2

handbuch-print.ps.bz2: handbuch.pdf
	pdf2ps handbuch.pdf
	psbook -s8 handbuch.ps > temp.ps
	pstops "2:0L@1(21.0cm,0)+1L@1(21.0cm,14.8cm)" temp.ps > handbuch-print.ps
	bzip2 handbuch-print.ps
	rm -f handbuch.ps temp.ps

.tex.dvi:
	if [ ! -f ${.PREFIX}.aux ] ; then touch ${.PREFIX}.aux ; fi
	cp ${.PREFIX}.aux ${.PREFIX}.aux.bak
	${LATEX} ${.IMPSRC} </dev/null
	if ! cmp -s ${.PREFIX}.aux ${.PREFIX}.aux.bak ; then \
		rm -f ${.TARGET}; \
		${LATEX} ${.IMPSRC} </dev/null; \
	fi
	rm -f ${.PREFIX}.aux.bak

.tex.pdf:
	if [ ! -f ${.PREFIX}.aux ] ; then touch ${.PREFIX}.aux ; fi
	cp ${.PREFIX}.aux ${.PREFIX}.aux.bak
	${PDFLATEX} ${.IMPSRC} </dev/null
	if ! cmp -s ${.PREFIX}.aux ${.PREFIX}.aux.bak ; then \
		rm -f ${.TARGET}; \
		${PDFLATEX} ${.IMPSRC} </dev/null; \
	fi
	rm -f ${.PREFIX}.aux.bak

.dvi.ps:
	${DVIPS} -f < ${.IMPSRC} > ${.TARGET}

.tex.rtf:
	${LATEX2RTF} -l ${.IMPSRC} > ${.TARGET}
	@rm -f temp.tmp

clean:
	@echo Cleaning...
	#@rm -f *.dvi *.log *.aux *.lj *.ps *.toc *.rtf *~ \#*\#
	@rm -f *.dvi *.log *.aux *.lj *.toc *.out *.rtf *~ \#*\# temp.ps

print:
	@if expr "`echo *.ps`" : '*\.ps' > /dev/null ; \
	then \
		if expr "`echo *.lj`" : '*\.lj' > /dev/null ; \
		then \
			echo "There's nothing to print here" ; \
		else \
			echo Gonna print `echo *.lj`, 3 seconds to interrupt ;\
			sleep 3 ; \
			lpr -Pfax *.lj ; \
		fi \
	else \
		echo Gonna print `echo *.ps`, 3 seconds to interrupt ; \
		sleep 3 ; \
		lpr -PPS *.ps ; \
	fi

