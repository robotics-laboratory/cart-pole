.PHONY: all

all: \
	cart_pole_pdf \
	radial_cart_pole_svg \
	linear_cart_pole_svg \

build_dir:
	mkdir -p build

cart_pole_pdf: build_dir
	cd tex && pdflatex -output-directory ../build main.tex
	cp build/main.pdf cart_pole.pdf

linear_cart_pole_svg: build_dir
	latex -output-directory build tex/linear_cart_pole.tex
	dvisvgm --no-fonts build/linear_cart_pole.dvi -o svg/linear_cart_pole.svg

radial_cart_pole_svg: build_dir
	latex -output-directory build tex/radial_cart_pole.tex
	dvisvgm --no-fonts build/radial_cart_pole.dvi -o svg/radial_cart_pole.svg

clean:
	rm -rf build
	cd tex && latexmk -CA
