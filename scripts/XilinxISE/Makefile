VGA_MODE=1024x768_128x64x256
TWO_COLOR_MODE=FALSE
PROJECT=Papilio_AVR8
PLATFORM=Papilio_Plus
PART=xc6slx9-3-tqg144

UCF_NAME=${PLATFORM}.ucf
UCF=../../sources/${UCF_NAME}

ifeq ($(TWO_COLOR_MODE),TRUE)
OUT_NAME=${PLATFORM}_AVR8_${VGA_MODE}_2C
else
OUT_NAME=${PLATFORM}_AVR8_${VGA_MODE}
endif

all: ${OUT_NAME}.bit

${PROJECT}.ngc:
	mkdir -p xst/projnav.tmp/
	@rm -f __tmp.prt __tmp.xst
	echo "-generics {DIS_DESC_NAME=\"${VGA_MODE}\" TWO_COLOR_ONLY=${TWO_COLOR_MODE}}" > __tmp.prt
	cat ${PROJECT}.xst __tmp.prt > __tmp.xst
	xst -intstyle ise -ifn __tmp.xst -ofn ${PROJECT}.syr
	@rm -f __tmp.prt __tmp.xst
        
${PROJECT}.ngd: ${PROJECT}.ngc
	ngdbuild -intstyle ise -dd _ngo -nt timestamp \
	-uc ${UCF} -p ${PART} ${PROJECT}.ngc ${PROJECT}.ngd

${PROJECT}_map.ncd: ${PROJECT}.ngd
	map -intstyle ise -p ${PART} \
	-w -detail -ir off -ignore_keep_hierarchy -pr b -timing -ol high -logic_opt on  \
	-o ${PROJECT}_map.ncd ${PROJECT}.ngd ${PROJECT}.pcf 

${PROJECT}.ncd: ${PROJECT}_map.ncd
	par -w -intstyle ise -ol high ${PROJECT}_map.ncd ${PROJECT}.ncd ${PROJECT}.pcf

${OUT_NAME}.bit: ${PROJECT}.ncd
	bitgen -f ${PROJECT}.ut ${PROJECT}.ncd
	mv -f ${PROJECT}.bit ${OUT_NAME}.bit

tidy:
	@rm -f ${PROJECT}.{ngc,ngd,ncd,_map.ncd,pcf}
	
clean:
	@rm -f ${PROJECT}.{ngc,ngd,ncd,_map.ncd,pcf}
	@rm -f ${OUT_NAME}.{bit,bin}

clean_all:
	@rm -f *.{ngc,ngd,ncd,_map.ncd,pcf}
	@rm -f *.{bit,bin}

	