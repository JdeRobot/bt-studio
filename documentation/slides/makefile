all: pdf

unibotics: unibotics.dvi
	dvips -t a4 -t landscape -f unibotics.dvi > unibotics.ps

unibotics.dvi: unibotics.tex
	latex unibotics.tex

clean:
	rm -f unibotics.dvi unibotics.ps unibotics.aux unibotics.pdf unibotics.log unibotics.out

doblepagina: unibotics.dvi unibotics a4caras

a4caras:
	psnup -4 unibotics.ps unibotics2.ps

pdf:
	pdflatex unibotics.tex

version:
	tar -cvzf nuevaversion.tgz img/ *.tex makefile *.sty *.jpg *.eps
