# Quick Install Guide

1. Clone the repository
```
git clone https://github.com/dasimagin/cart_pole
cd cart_pole
```
2. Install python requirements ([what is poetry?](https://python-poetry.org/docs/))
```
poetry install --no-root
# OR
pip install -r requirements-dev.txt
```
3. Install [pre-commit hooks](https://pre-commit.com/)
```
pre-commit install
```
4. Run tests
```
pytest
```
