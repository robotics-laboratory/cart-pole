.PHONY: all

all: \
	derivation_cart_pole_pdf \
	radial_cart_pole_svg \
	classic_cart_pole_svg

build_dir:
	mkdir -p build

derivation_cart_pole_pdf: build_dir
	cd tex && pdflatex -output-directory ../build derivation_cart_pole.tex
	cp build/derivation_cart_pole.pdf pdf/derivation_cart_pole.pdf

classic_cart_pole_svg: build_dir
	latex -output-directory build tex/classic_cart_pole.tex
	dvisvgm --no-fonts build/classic_cart_pole.dvi -o svg/*

radial_cart_pole_svg: build_dir
	latex -output-directory build tex/radial_cart_pole.tex
	dvisvgm --no-fonts build/radial_cart_pole.dvi -o svg/*