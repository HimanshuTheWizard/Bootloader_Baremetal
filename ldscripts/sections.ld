/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * Default linker script for Cortex-M (it includes specifics for 
 * STM32F[34]xx).
 * 
 * To make use of the multi-region initialisations, define
 * OS_INCLUDE_STARTUP_INIT_MULTIPLE_RAM_SECTIONS for the _startup.c file.
 */

/*
 * The '__stack' definition is required by crt0, do not remove it.
 */
__stack = ORIGIN(RAM) + LENGTH(RAM);

_estack = __stack; 	/* STM specific definition */

/*
 * Default stack sizes.
 * These are used by the startup in order to allocate stacks 
 * for the different modes.
 */

__Main_Stack_Size = 1024 ;

PROVIDE ( _Main_Stack_Size = __Main_Stack_Size ) ;

__Main_Stack_Limit = __stack  - __Main_Stack_Size ;

/* "PROVIDE" allows to easily override these values from an 
 * object file or the command line. */
PROVIDE ( _Main_Stack_Limit = __Main_Stack_Limit ) ;

/*
 * There will be a link error if there is not this amount of 
 * RAM free at the end. 
 */
_Minimum_Stack_Size = 256 ;

/*
 * Default heap definitions.
 * The heap start immediately after the last statically allocated 
 * .sbss/.noinit section, and extends up to the main stack limit.
 */
PROVIDE ( _Heap_Begin = _end_noinit ) ;
PROVIDE ( _Heap_Limit = __stack - __Main_Stack_Size ) ;

/* 
 * The entry point is informative, for debuggers and simulators,
 * since the Cortex-M vector points to it anyway.
 */
ENTRY(_start)


/* Sections Definitions */

SECTIONS
{
    /*
     * For Cortex-M devices, the beginning of the startup code is stored in
     * the .isr_vector section, which goes to FLASH. 
     */
    .isr_vector : ALIGN(4)
    {
        FILL(0xFF)
        
        __vectors_start = ABSOLUTE(.) ;
        __vectors_start__ = ABSOLUTE(.) ; /* STM specific definition */
        KEEP(*(.isr_vector))     	/* Interrupt vectors */
        
		KEEP(*(.cfmconfig))			/* Freescale configuration words */   
		     
        /* 
         * This section is here for convenience, to store the
         * startup code at the beginning of the flash area, hoping that
         * this will increase the readability of the listing.
         */
        *(.after_vectors .after_vectors.*)	/* Startup code and ISR */

    } >FLASH

    .inits : ALIGN(4)
    {
        /* 
         * Memory regions initialisation arrays.
         *
         * Thee are two kinds of arrays for each RAM region, one for 
         * data and one for bss. Each is iterrated at startup and the   
         * region initialisation is performed.
         * 
         * The data array includes:
         * - from (LOADADDR())
         * - region_begin (ADDR())
         * - region_end (ADDR()+SIZEOF())
         *
         * The bss array includes:
         * - region_begin (ADDR())
         * - region_end (ADDR()+SIZEOF())
         *
         * WARNING: It is mandatory that the regions are word aligned, 
         * since the initialisation code works only on words.
         */
         
        __data_regions_array_start = .;
        
        LONG(LOADADDR(.data));
        LONG(ADDR(.data));
        LONG(ADDR(.data)+SIZEOF(.data));
        
        LONG(LOADADDR(.data_CCMRAM));
        LONG(ADDR(.data_CCMRAM));
        LONG(ADDR(.data_CCMRAM)+SIZEOF(.data_CCMRAM));
        
        __data_regions_array_end = .;
        
        __bss_regions_array_start = .;
        
        LONG(ADDR(.bss));
        LONG(ADDR(.bss)+SIZEOF(.bss));
        
        LONG(ADDR(.bss_CCMRAM));
        LONG(ADDR(.bss_CCMRAM)+SIZEOF(.bss_CCMRAM));
        
        __bss_regions_array_end = .;

        /* End of memory regions initialisation arrays. */
    
        /*
         * These are the old initialisation sections, intended to contain
         * naked code, with the prologue/epilogue added by crti.o/crtn.o
         * when linking with startup files. The standalone startup code
         * currently does not run these, better use the init arrays below.
         */
		KEEP(*(.init))
		KEEP(*(.fini))

		. = ALIGN(4);

		/*
         * The preinit code, i.e. an array of pointers to initialisation 
         * functions to be performed before constructors.
         */
		PROVIDE_HIDDEN (__preinit_array_start = .);
        
        /*
         * Used to run the SystemInit() before anything else.
         */
		KEEP(*(.preinit_array_sysinit .preinit_array_sysinit.*))
        
        /* 
         * Used for other platform inits.
         */
		KEEP(*(.preinit_array_platform .preinit_array_platform.*))
        
        /*
         * The application inits. If you need to enforce some order in 
         * execution, create new sections, as before.
         */
		KEEP(*(.preinit_array .preinit_array.*))

		PROVIDE_HIDDEN (__preinit_array_end = .);

		. = ALIGN(4);

		/*
         * The init code, i.e. an array of pointers to static constructors.
         */
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		PROVIDE_HIDDEN (__init_array_end = .);

		. = ALIGN(4);

		/*
         * The fini code, i.e. an array of pointers to static destructors.
         */
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP(*(SORT(.fini_array.*)))
		KEEP(*(.fini_array))
		PROVIDE_HIDDEN (__fini_array_end = .);

    } >FLASH

    /*
     * For some STRx devices, the beginning of the startup code
     * is stored in the .flashtext section, which goes to FLASH.
     */
    .flashtext : ALIGN(4)
    {
        *(.flashtext .flashtext.*)	/* Startup code */
    } >FLASH
 
    
    /*
     * The program code is stored in the .text section, 
     * which goes to FLASH.
     */
    .text : ALIGN(4)
    {
        *(.text .text.*)			/* all remaining code */
 
 		/* read-only data (constants) */
        *(.rodata .rodata.* .constdata .constdata.*) 		

        *(vtable)					/* C++ virtual tables */

		KEEP(*(.eh_frame*))

		/*
		 * Stub sections generated by the linker, to glue together 
		 * ARM and Thumb code. .glue_7 is used for ARM code calling 
		 * Thumb code, and .glue_7t is used for Thumb code calling 
		 * ARM code. Apparently always generated by the linker, for some
		 * architectures, so better leave them here.
		 */
        *(.glue_7)
        *(.glue_7t)

    } >FLASH

    .boot_metadata (NOLOAD) : 
    {
        . = ALIGN(4);
        KEEP(*(.boot_metadata))
        . = ALIGN(4);
    } >FLASH_METADATA


	/* ARM magic sections */
	.ARM.extab : ALIGN(4)
   	{
       *(.ARM.extab* .gnu.linkonce.armextab.*)
   	} > FLASH
   	
    . = ALIGN(4);
   	__exidx_start = .;   	
   	.ARM.exidx : ALIGN(4)
   	{
       *(.ARM.exidx* .gnu.linkonce.armexidx.*)
   	} > FLASH
   	__exidx_end = .;
   	
    . = ALIGN(4);
    _etext = .;
    __etext = .;
    
    /* MEMORY_ARRAY */
    /*
    .ROarraySection :
    {
     	*(.ROarraySection .ROarraySection.*)                          
    } >MEMORY_ARRAY
    */

	/*
	 * The secondary initialised data section.
	 */
    .data_CCMRAM : ALIGN(4)
    {
       FILL(0xFF)
       *(.data.CCMRAM .data.CCMRAM.*)
       . = ALIGN(4) ;
    } > CCMRAM AT>FLASH

	/* 
     * This address is used by the startup code to 
     * initialise the .data section.
     */
    _sidata = LOADADDR(.data);

    /*
     * The initialised data section.
     *
     * The program executes knowing that the data is in the RAM
     * but the loader puts the initial values in the FLASH (inidata).
     * It is one task of the startup to copy the initial values from 
     * FLASH to RAM.
     */
    .data : ALIGN(4)
    {
    	FILL(0xFF)
        /* This is used by the startup code to initialise the .data section */
        _sdata = . ;        	/* STM specific definition */
        __data_start__ = . ;
		*(.data_begin .data_begin.*)

		*(.data .data.*)
		
		*(.data_end .data_end.*)
	    . = ALIGN(4);

	    /* This is used by the startup code to initialise the .data section */
        _edata = . ;        	/* STM specific definition */
        __data_end__ = . ;

    } >RAM AT>FLASH
    
    /*
     * The uninitialised data sections. NOLOAD is used to avoid
     * the "section `.bss' type changed to PROGBITS" warning
     */
     
    /* The secondary uninitialised data section. */
	.bss_CCMRAM (NOLOAD) : ALIGN(4)
	{
		*(.bss.CCMRAM .bss.CCMRAM.*)
	} > CCMRAM

    /* The primary uninitialised data section. */
    .bss (NOLOAD) : ALIGN(4)
    {
        __bss_start__ = .;     	/* standard newlib definition */
        _sbss = .;              /* STM specific definition */
        *(.bss_begin .bss_begin.*)

        *(.bss .bss.*)
        *(COMMON)
        
        *(.bss_end .bss_end.*)
	    . = ALIGN(4);
        __bss_end__ = .;        /* standard newlib definition */
        _ebss = . ;             /* STM specific definition */
    } >RAM

    .noinit_CCMRAM (NOLOAD) : ALIGN(4)
    {
        *(.noinit.CCMRAM .noinit.CCMRAM.*)         
    } > CCMRAM
    
    .noinit (NOLOAD) : ALIGN(4)
    {
        _noinit = .;
        
        *(.noinit .noinit.*) 
        
         . = ALIGN(4) ;
        _end_noinit = .;   
    } > RAM
    
    /* Mandatory to be word aligned, _sbrk assumes this */
    PROVIDE ( end = _end_noinit ); /* was _ebss */
    PROVIDE ( _end = _end_noinit );
    PROVIDE ( __end = _end_noinit );
    PROVIDE ( __end__ = _end_noinit );
    
    /*
     * Used for validation only, do not allocate anything here!
     *
     * This is just to check that there is enough RAM left for the Main
     * stack. It should generate an error if it's full.
     */
    ._check_stack : ALIGN(4)
    {
        . = . + _Minimum_Stack_Size ;
    } >RAM
    
    /*
     * The FLASH Bank1.
     * The C or assembly source must explicitly place the code 
     * or data there using the "section" attribute.
     */
    .b1text : ALIGN(4)
    {
        *(.b1text)                   /* remaining code */
        *(.b1rodata)                 /* read-only data (constants) */
        *(.b1rodata.*)
    } >FLASHB1
    
    /*
     * The EXTMEM.
     * The C or assembly source must explicitly place the code or data there
     * using the "section" attribute.
     */

    /* EXTMEM Bank0 */
    .eb0text : ALIGN(4)
    {
        *(.eb0text)                   /* remaining code */
        *(.eb0rodata)                 /* read-only data (constants) */
        *(.eb0rodata.*)
    } >EXTMEMB0
    
    /* EXTMEM Bank1 */
    .eb1text : ALIGN(4)
    {
        *(.eb1text)                   /* remaining code */
        *(.eb1rodata)                 /* read-only data (constants) */
        *(.eb1rodata.*)
    } >EXTMEMB1
    
    /* EXTMEM Bank2 */
    .eb2text : ALIGN(4)
    {
        *(.eb2text)                   /* remaining code */
        *(.eb2rodata)                 /* read-only data (constants) */
        *(.eb2rodata.*)
    } >EXTMEMB2
    
    /* EXTMEM Bank0 */
    .eb3text : ALIGN(4)
    {
        *(.eb3text)                   /* remaining code */
        *(.eb3rodata)                 /* read-only data (constants) */
        *(.eb3rodata.*)
    } >EXTMEMB3
   

    /* After that there are only debugging sections. */
    
    /* This can remove the debugging information from the standard libraries */    
    /* 
    DISCARD :
    {
     libc.a ( * )
     libm.a ( * )
     libgcc.a ( * )
     }
     */
  
    /* Stabs debugging sections.  */
    .stab          0 : { *(.stab) }
    .stabstr       0 : { *(.stabstr) }
    .stab.excl     0 : { *(.stab.excl) }
    .stab.exclstr  0 : { *(.stab.exclstr) }
    .stab.index    0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment       0 : { *(.comment) }
    /*
     * DWARF debug sections.
     * Symbols in the DWARF debugging sections are relative to the beginning
     * of the section so we begin them at 0.  
     */
    /* DWARF 1 */
    .debug          0 : { *(.debug) }
    .line           0 : { *(.line) }
    /* GNU DWARF 1 extensions */
    .debug_srcinfo  0 : { *(.debug_srcinfo) }
    .debug_sfnames  0 : { *(.debug_sfnames) }
    /* DWARF 1.1 and DWARF 2 */
    .debug_aranges  0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    /* DWARF 2 */
    .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    /* SGI/MIPS DWARF 2 extensions */
    .debug_weaknames 0 : { *(.debug_weaknames) }
    .debug_funcnames 0 : { *(.debug_funcnames) }
    .debug_typenames 0 : { *(.debug_typenames) }
    .debug_varnames  0 : { *(.debug_varnames) }    
}
