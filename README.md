# coppelia-robots
Проект реализован и выложен на GitHub 15.11.2022

## Оглавление
+ [Описание](#Описание)
+ [Запуск проекта](#Запуск-проекта)
+ [Скриншоты симуляции](#Скриншоты-симуляции)
+ [Детали](#Детали)
+ [Спасибо за внимание, дорогой читатель! Вкусных пряников, друг :)](#спасибо-за-внимание-дорогой-читатель-вкусных-пряников-друг-)

## Описание
Привет, друг! :smiley:

В этом проекте, который я реализовал совместно с моими одногруппниками по вузу, представлена **CoppeliaSim** сцена лабиринта (англ. *scene*). В эту сцену включены четыре виртуальных мобильных робота: два робота с контактными датчиками на их бамперах (англ. *bumper force sensors* :muscle:), один робот левой руки и один робот правой руки, на которые установлены по три дистанционных датчика (англ. *proximity sensors*).

Хоть задание преподавателя в этом проекте и не выполнено (нужно было реализовать именно взаимодействующих роботов, исследующих лабиринт по оптимальным маршрутам каждый :running:), но сил было вложено в работу много, поэтому я её выкладываю сюда на память.

**Спасибо ребятам (одногруппникам КРМО-01-22 Егору, Лёхе, Ярику и Даниле :boom::star:), сотрудничающими со мной на протяжении выполнения этой работы — без их помощи и поддержки я бы не написал весь этот код!**

## Запуск проекта
Для запуска этого проекта Вам, читатель, необходима среда моделирования роботов CoppeliaSim (в прошлом V-REP; далее просто — Коппелиа), под которой и сделан этот проект. Код Питона этого репозитория запустится только в этой среде из-за некоторых специфических особенностей языковой обработки Коппелии (её препроцессора) и наличия объектов с её сцены (включая глобальный объект **sim**, стены лабиринта и роботов).

Кроме того, нужна версия *не ниже 4.3.0* программы!:fire: В этой версии и был добавлен Питон.

После того, как Вы установите среду моделирования CoppeliaSim, откройте предварительно скачанную сцену репозитория и запустите симуляцию.

| :heavy_exclamation_mark:Так как Коппелиа использует внешний интерпретатор Питона — python.exe — то необходимо его установить, чтобы эта сцена успешно начала симулироваться.|
|:----:|

| :pencil:Но можно, по Вашему желанию, переписать код сцены на **Lua**, который, кстати говоря, обработается Коппелией ещё быстрее и проще, нежели чем Python, как разработчики пишут в документации к симулятору. |
|:----:|

Папку репозитория *"code"* качать необязательно: в ней лежат скрипты роботов обсуждаемой сцены, а они уже и так в сцене.

## Скриншоты симуляции
Изначальный вид сцены CoppeliaSim:

![image](https://user-images.githubusercontent.com/80912103/201990844-e702ba5f-837c-4e8a-b647-13bc54fcc4f6.png)

Запуск симуляции: роботы ездят по лабиринту и исследуют его:

![image](https://user-images.githubusercontent.com/80912103/201991085-3dc9f1d1-f84d-49f6-a689-d2458221a181.png)

**Карта**, которую роботы рисуют вместе, используя свои датчики (это окно создаётся благодаря модулю *graphics.py*):

![graphics](https://user-images.githubusercontent.com/80912103/201992350-67e0fbcf-c48a-4df9-91de-46e5b5c6e224.png)

Три робота сцены:
+ по середине находится робот со **случайным** алгоритмом исследования лабиринта (использует питоновский модуль *random.py*) (ещё один такой находится выше вне скриншота),
+ слева находится робот, применяющий правило **левой руки** для объезда лабиринта,
+ а справа — робот **правой руки**:

![robots](https://user-images.githubusercontent.com/80912103/201992747-7cb7db63-8a85-464c-ac03-a1fe2614e714.png)

## Детали
Для примера будет описан код робота правой руки `dr12[3]` :point_right:. Он должен ехать по правым стенам лабиринта, ориентируясь на свой правый датчик `Proximity_sensor[2]` (выделен синим цветом):

![dr12_3](https://user-images.githubusercontent.com/80912103/201995928-9653bf32-d8fe-43c7-86e2-12887bdc6f2e.png)

Бампер у робота удалён. У него есть тело ('body') (см. левое меню на скриншоте выше — снизу), левый и правый моторы (`leftJoint_` и `rightJoint_`) и три дистанционных датчика, повёрнутых направо, вперёд и налево — для общей корректной ориентации в пространстве.

**Скрипт** этого робота можно открыть для редактирования и просмотра, дважды нажав на иконку *листа* правее самого объекта `dr12[3]` в левом навигационном меню CoppeliaSim: ![script](https://user-images.githubusercontent.com/80912103/201997697-2ce732f3-31c5-481b-8316-2b31e106f30b.png).

В начале скрипта **подключается бибилотека** `random.py` (она используется и в таком роботе) и объявляются **глобальные переменные**:
```python
#python
include random  ## обратите внимание, что не "import", а "include" в Коппелии!

## GLOBALS:
robot, leftJointHandle, rightJointHandle, changeOrientationTime, graph, X, Y = 0, 0, 0, 0, 0, 0, 0
proximitySensors = 0                        ## datchiki
baseVelocity = 5                            ## skorost
blockTime, Right90Time, flag = 0, 0, False  ## to do turns properly
N = 4                                       ## robots amount
```

Далее идёт череда нескольких **вспомогательных** функций, которые поворачивают робота в разные стороны и определяют его ориентацию в пространстве:
```python
<...>

## to correctly draw detected points on map!
def orientation(Rxyz):
    PI = math.pi
    if (Rxyz > -PI/4) and (Rxyz <= PI/4):
        return 'RIGHT'
    elif (Rxyz > PI/4) and (Rxyz <= 3*PI/4):
        return 'TOP'
    elif ((Rxyz > 3*PI/4) and (Rxyz <= 5*PI/4)) or ((Rxyz < -3*PI/4) and (Rxyz >= -5*PI/4)):
        return 'LEFT'
    elif (Rxyz < -PI/4) and (Rxyz >= -3*PI/4):
        return 'BOTTOM'

<...>
```

После этого идёт функция **инициализации** :sun_with_face: всех параметров робота:
```python
##===========================================================
## Initialize a robot and his position graphics:
def init():
    global robot, leftJointHandle, rightJointHandle, proximitySensors
    robot = sim.getObject('.')
    leftJointHandle = sim.getObject("./leftJoint_")
    rightJointHandle = sim.getObject("./rightJoint_")
    
    proximitySensors = dict()
    proximitySensors.update(front=sim.getObject('./Proximity_sensor', {'index': 0}))
    proximitySensors.update(left=sim.getObject('./Proximity_sensor', {'index': 1}))
    proximitySensors.update(right=sim.getObject('./Proximity_sensor', {'index': 2}))
    
    goAhead()
    
    ## Graph initialization:
    global graph, X, Y
    graph = sim.getObject('/Graph', {'index': 3})
    X = sim.addGraphStream(graph, 'X coord', 'm')
    Y = sim.addGraphStream(graph, 'Y coord', 'm')
    sim.addGraphCurve(graph, 'XY Robot\'s Position', 2, [X,Y], [0,0], '')
```

В функции `init()` создаются ссылки на:
+ объект самого робота (переменная `robot`),
+ объекты левого и правого моторов (переменные `leftJointHandle` и `rightJointHandle`),
+ создаётся Python-словарь `dict proximitySensors` для трёх датчиков робота,
+ вызывается функция `goAhead()` движения робота вперёд,
+ в конце создаётся встроенный в симулятор **граф**, который будет показывать текущее местоположение робота на сцене (координаты `x` и `y`).

Затем идёт центральная функция `actuation()`, которая определяет поведение робота в ходе симуляции. Робот ориентируется в зависимости от расстояния от правой, передней и левой стен лабиринта. Расстояние от стен измеряют соответствующие датчики. Например, 94-я строка кода:

```python
<...>
detectedObjectHandle, detectedPoint, distance, result = sim.handleProximitySensor(proximitySensors['front'])[-2::-1]
<...>
```
выполняет встроенную функцию Коппелии `sim.handleProximitySensor(int sensorHandle)` для запуска **измерения расстояния :computer: и считывания результата** с переднего (англ. *front*) датчика. Если переменная `result` при этом получит значение `0`, то это говорит о том, что стена замечена не была спереди. (Если `-1`, то возникла какая-то ошибка.) А если `1`, то стена замечена и соответствующие дистанция, координаты замеченной точки стены (относительно системы отсчёта самого датчика, а не сцены) и ссылка на выявленный объект будут в переменных `distance`, `detectedPoint` и `detectedObjectHandle`.

Далее исключаются из отмечания на карте **другие роботы**, (если замечены были они, а не стена лабиринта; *почему-то это не работает, по сути — роботы всё равно отмечаются на карте* :worried:):

```python
for i in range(N):
    if i != 2:
        if detectedObjectHandleRight == sim.getObject('/dr12', {'index': i}):
            resultRight = -1
        if detectedObjectHandle == sim.getObject('/dr12', {'index': i}):
            result = -1
        if detectedObjectHandleLeft == sim.getObject('/dr12', {'index': i}):
            resultLeft = -1
```

Дальше объявляются локальные переменные `DISTMAX` и `DISTMIN`, которые обозначают **максимальное и минимальное расстояние** езды робота от *правой* стены. Т.е. если сейчас расстояние от правой стены больше `DISTMAX`, то робот поворачивает немного правее, чтобы приблизиться к ней. То же с минимальной дистанцией.

При въезде в **тупик** робот разворачивается на 180 градусов функцией `turnAround()`.
Ещё ниже (124-я и 125-я строки) следует немаловажный фрагмент кода:

```python
elif (resultLeft == 0) and (resultRight == 1) and (result == 1):
    turnLeft(math.pi/2)
```
который повернёт робота налево, если справа и спереди тупик есть, а слева его нет.

Далее идёт вычисление текущей **ориентации** робота на сцене:

```python
## Draw detected points on map:
Rxyz = sim.getObjectOrientation(robot, -1)                                 ## rotations floats about axes Z, Y and X respectively
Rz = list(sim.alphaBetaGammaToYawPitchRoll(Rxyz[0], Rxyz[1], Rxyz[2]))[0]  ## current robot rotation about !Z axe!
```
для правильного отрисовывания замеченных точек лабиринта на карте.

И **посылка сигнала** *"РИСОВАТЬ ТОЧКУ"* другому скрипту, отвечающему за карту — скрипту объекта `diningChair`:

```python
<...>
sim.setStringSignal('FEEL', str(position[0]) + ' ' + str(position[1]))
<...>
```

Последняя функция скрипта :relaxed:

```python
def sysCall_thread():
    ## Python thread initialization:
    sim.setThreadAutomaticSwitch(True)
    init()
    
    ## Python thread main loop:
    while not sim.getThreadExistRequest():
        actuation()
```
создаёт отдельный дочерний питоновский процесс для выполнения всего этого скрипта. Эту функцию и запускает в самом начале главный процесс Коппелии. Можно сказать, что этот скрипт — *child threaded script* в терминологии разработчиков CoppeliaSim :bowtie:.

## Спасибо за внимание, дорогой читатель! Вкусных пряников, друг :)
