# Gen3DFactoryModels
Применение генеративных сетей для создания 3D-моделей производственных объектов
## Предлагаемое решение
1. Реконструкция сцены с сегментацией. Gaussian Splatting для создания 3D-представления сцены и одновременное применение Gaussian Grouping для сегментации объектов внутри этой сцены на уровне отдельных гауссиан. 
2. Изоляция и рендеринг объектов. Выделение групп гауссиан, соответствующих каждому интересующему объекту, и генерация изображений (рендеров) этих изолированных объектов с разных ракурсов.
3. Генерация твердотельных моделей. Использование рендеров каждого объекта как входных данных для генеративной модели (Trellis или TripoSR) для получения его 3D-модели (сетки).
![image](https://github.com/user-attachments/assets/4996e881-81d3-4452-a633-71deab3968f9)
## Полученные примеры
![image](https://github.com/user-attachments/assets/b0efd97d-e636-4bc3-92c9-874f264f1074)

## Модель Trellis

### Примеры работы

Входное изображение:  
![03190075dda911efabc606122d8f209b_1](https://github.com/user-attachments/assets/0d090412-b38a-43ad-ba05-89b6ec2bde2c)  
  
Полученная модель:  
![рабочий модель](https://github.com/user-attachments/assets/15fb186e-8af0-42da-ada4-7252d8ae3423)  

Входное изображение:  
![0e32aa33ddaa11efacf526eac4d45a1b_1](https://github.com/user-attachments/assets/20fb6698-85eb-46bc-8bbf-c8ba932a95d9)  

Полученная модель:  
![станок модель](https://github.com/user-attachments/assets/9914fa4e-3d67-4597-bf78-151d9859cdcc)  

Реконструкция по нескольким изображениям:  
https://github.com/Hedy-dev/GaussianSplatting_3DR_SLAM
