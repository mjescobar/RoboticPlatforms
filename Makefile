git:
	sh clean.sh
	@cd RobotLIB; make cleandocs
	git add --all
	git commit -m "$(commit)"
	git push