# Script para plotar a saida do sistema usando o GNU plot

reset
set grid
#set title "Resultado da Simulacao"
set ylabel "Valor"

plot 'traj' u 1:2 title "real" w lines, 'traj' u 4:5 title "referÍncia" w lines

pause -1




