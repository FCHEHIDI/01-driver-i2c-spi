/* Linker script pour le STM32F411xE
 *
 * Flash : 512 KiB  à 0x0800_0000  (programme + données constantes)
 * RAM   : 128 KiB  à 0x2000_0000  (stack, heap si présent, données)
 *
 * cortex-m-rt lit ce fichier au link et place :
 *   - ORIGIN(FLASH) → vecteurs d'interruption + code
 *   - ORIGIN(RAM)   → .data, .bss, stack
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 512K
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}

/* Taille de la pile (stack) : 8 KiB – ajuster si besoin */
_stack_size = 0x2000;
