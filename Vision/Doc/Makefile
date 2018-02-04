DIAGRAMS=$(wildcard diagrams/*.tex)

%.pdf: %.tex
	pdflatex $< && pdflatex $< && pdflatex $<

all: PipelineDescription.pdf

PipelineDescription.pdf: ${DIAGRAMS}

clean:
	rm -f *.aux *.snm *.toc *.log

mrproper: clean
	rm PipelineDescription.pdf
