# Script para plotar a saida do sistema usando o GNU plot

reset
set grid
#set title "Resultado da Simulacao"
set ylabel "Valor"

plot 'traj' u 1:2 title "sa�da controlada" w lines, 'traj' u 4:5 title "refer�ncia" w lines

pause -1




