savedcmd_ov9234.mod := printf '%s\n'   ov9234.o | awk '!x[$$0]++ { print("./"$$0) }' > ov9234.mod
