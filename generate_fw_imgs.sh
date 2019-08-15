#!/bin/sh

FW_PREFIX="koryuu-fw"
TARGETS="hex debug debug2 no_panic no_autoreset"
ARCHIVE_NAME="${FW_PREFIX}_images.zip"
HEX_TARGETS=""

SEVENZIP_CANDIDATES="7z 7za"
SEVENZIP=""
for cnd in ${SEVENZIP_CANDIDATES}; do
	SEVENZIP="$(which ${cnd})"
	[ -n "${SEVENZIP}" -a -x "${SEVENZIP}" ] && break
	SEVENZIP=""
done

[ -z "${SEVENZIP}" ] && echo "7zip not found!" && exit 1

COMPRESS="${SEVENZIP} a -bb1 -tzip -mx=9 ${ARCHIVE_NAME}"

for target in ${TARGETS}; do
	[ "x${target}" = "xhex" ] && target=default
	HEX_TARGETS="${HEX_TARGETS} ${FW_PREFIX}_${target}.hex"
done

rm -f ${HEX_TARGETS} "${ARCHIVE_NAME}"

# Ugly repetition of the HEX_TARGETS logic... ideas anyone?
for target in ${TARGETS}; do
	make "build_${target}" || exit 1
	[ "x${target}" = "xhex" ] && target=default
	mv "${FW_PREFIX}.hex" "${FW_PREFIX}_${target}.hex" || exit 1
done

rm -f "${FW_PREFIX}.hex"
ln -s "${FW_PREFIX}_default.hex" "${FW_PREFIX}.hex"
ls -al ${HEX_TARGETS}
${COMPRESS} ${HEX_TARGETS}
