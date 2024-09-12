all: paper.pdf

paper.aux: paper.tex
	pdflatex -interaction=scrollmode paper

paper.bbl: paper.aux paper.bib
	bibtex paper.aux

paper.pdf: paper.tex paper.bbl
	pdflatex -interaction=batchmode paper