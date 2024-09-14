# ROS2
repo de prueba

pasos para crear linkear un repo con tu ws

Configurar credenciales

git config --global user.name "Your Name"
git config --global user.email "your-email@example.com"

crea una ssh key para el repo

ssh-keygen -t ed25519 -C "your-email@example.com"

copia lo que te crea en la caja de texto con esto

cat ~/.ssh/id_ed25519.pub

Iniciar git en tu ws

git init
git add .
git remote set-url origin git@github.com:yourusername/repositoryname.git
git push -u origin main

