clean:
	@cd Experiments; make clean
	@cd RobotLIB; make clean

git:
	make clean
	git add --all
	git commit -m "$(commit)"
	git push