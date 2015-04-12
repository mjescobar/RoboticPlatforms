clean:
	@cd Experiments; make clean
	@cd RobotLIB; make clean

git:
	make clean
	@cd RobotLIB; make cleandocs
	git add --all
	git commit -m "$(commit)"
	git push