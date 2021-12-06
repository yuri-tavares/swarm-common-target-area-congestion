##Remove arquivos de log invalidos 

#Remove arquivos que possuam apenas 0s
rm `grep ^0$ -Rl . | grep log`;

#remove logs que possuam valores de execução maior que 100000
rm `grep 10000 -Rl . | grep log`;

#remove arquivos vazios
find . -size 0 -delete;
