# Script para plotar a saida do sistema usando o GNU plot

reset
set grid
#set title "Resultado da Simulacao"
set ylabel "Valor"

plot 'quadrado_sem_k' u 1:2 title "real" w lines, 'quadrado_sem_k' u 4:5 title "referência" w lines

pause -1




