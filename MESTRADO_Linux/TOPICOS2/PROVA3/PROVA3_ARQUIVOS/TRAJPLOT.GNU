# Script para plotar a saida do sistema usando o GNU plot

reset
set grid
#set title "Resultado da Simulacao"
set ylabel "Valor"

plot 'xxx_ctrl' u 15:16 title "estado controlado" w lines, 'xxx_ctrl' u 12:13 title "referência" w lines

pause -1




