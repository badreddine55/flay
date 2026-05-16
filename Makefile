
FILE ?= map.txt
 
install:
	pip install pygame
 
run:
	python3 main.py $(FILE)
 
debug:
	python3 -m pdb main.py $(FILE)
 
lint:
	flake8 .
	mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports --disallow-untyped-defs --check-untyped-defs
 
lint-strict:
	flake8 .
	mypy . --strict
 
clean:
	rm -rf __pycache__ .mypy_cache