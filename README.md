 🐢 turtlesim_figure_drawer
**turtlesim_figure_drawer** est un projet pédagogique basé sur ROS2 (Robot Operating System 2) utilisant le package `turtlesim`.  
Il permet de tracer automatiquement plusieurs figures géométriques (carré, rectangle, triangle, losange) à l’aide d’un nœud Python personnalisé.

---
## Aperçu

Le robot turtle trace successivement :

✅ Un carré  
➡️ Se décale légèrement  
✅ Un rectangle  
🔺 Un triangle  
💎 Un losange  

## 🚀 Fonctionnalités

- Nœud ROS2 en Python
- Utilisation de :
  - Topics (`/turtle1/cmd_vel`, `/turtle1/pose`)
  - Timers ROS2
  - Enum pour la gestion d'état
- Publication d’un message `Status` personnalisé (interface ROS2)
- Contrôle autonome de la tortue pour dessiner différentes figures

---
