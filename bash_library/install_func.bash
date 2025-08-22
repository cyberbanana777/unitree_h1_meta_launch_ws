#!/bin/bash

# Функция для вывода статусных сообщений
print_status() {
    echo -e "\n\033[1;34m==>\033[0m $1"
}

# Функция для вывода сообщений об успехе
print_success() {
    echo -e "\033[1;32m[✓]\033[0m $1"
}

# Функция для вывода сообщений об ошибке
print_error() {
    echo -e "\033[1;31m[✗]\033[0m $1" >&2
    exit 1
}

main_install_function() {
    # Главный заголовок
    echo -e "\n\033[1;35m======================================="
    echo " Настройка рабочего пространства $1 "
    echo -e "=======================================\033[0m"

    # Сохраняем начальный путь
    start_path=$(pwd)
    print_status "Текущая директория сохранена: $start_path"

    # Создаем рабочее пространство
    print_status "Создаем рабочее пространство..."
    target_path="../../../.."
    mkdir -p "$target_path/$1/src" || print_error "Ошибка создания директорий"

    # Добавляем стартовый путь для updater-script в .bashrc
    cd $target_path || print_error "Ошибка перехода в директорию"
    print_status "Добавляем стартовый путь для updater-script в .bashrc..."
    line_to_add="export UNITREE_REPOS_ROOT='$(pwd)'"
    if ! grep -qxF "$line_to_add" ~/.bashrc; then
        echo "$line_to_add" >> ~/.bashrc
        print_success "Стартовый путь для updater-script добавлен в ~/.bashrc"
    else
        print_success "Стартовый путь для updater-script уже присутствует в ~/.bashrc"
    fi

    # Добавляем autosource для updater-script в .bashrc
    print_status "Добавляем autosource для updater-script в .bashrc..."
    line_to_add="source \"$(pwd)/unitree_h1_meta_launch_ws/src/bash_library/update_func.bash\""
    if ! grep -qxF "$line_to_add" ~/.bashrc; then
        echo "$line_to_add" >> ~/.bashrc
        print_success "autosource для updater-script добавлен в ~/.bashrc"
    else
        print_success "autosource для updater-script уже присутствует в ~/.bashrc"
    fi

    # Переходим в рабочую директорию
    cd "$1" || print_error "Ошибка перехода в директорию"
    print_success "Рабочая директория успешно создана: $(pwd)"

    cd "src" || print_error "Ошибка перехода в директорию"

    # Клонируем репозиторий
    print_status "Клонируем репозиторий..."
    git clone https://github.com/cyberbanana777/$1.git . \
        || print_error "Ошибка клонирования репозитория"
    print_success "Репозиторий успешно склонирован"

    # Устанавливаем зависимости
    print_status "Устанавливаем зависимости..."
    if [ -f "install_dependensies.bash" ]; then
        chmod +x install_dependensies.bash
        ./install_dependensies.bash || print_error "Ошибка установки зависимостей"
        print_success "Зависимости успешно установлены"
    else
        print_error "Файл зависимостей не найден: install_dependensies.bash"
    fi

    # Собираем пакеты
    print_status "Собираем пакеты..."
    cd .. || print_error "Ошибка перехода в корень рабочего пространства"
    colcon build || print_error "Ошибка сборки пакетов"
    print_success "Пакеты успешно собраны"

    # Настраиваем окружение
    print_status "Настраиваем окружение..."
    source install/setup.bash || print_error "Ошибка настройки окружения"
    print_success "Окружение настроено для текущей сессии"

    # Добавляем в .bashrc
    print_status "Добавляем автонастройку в .bashrc..."
    line_to_add="source \"$(pwd)/install/setup.bash\""
    if ! grep -qxF "$line_to_add" ~/.bashrc; then
        echo "$line_to_add" >> ~/.bashrc
        print_success "Автонастройка добавлена в ~/.bashrc"
    else
        print_success "Автонастройка уже присутствует в ~/.bashrc"
    fi

    # Возвращаемся в исходную директорию
    cd "$start_path" || print_error "Ошибка возврата в исходную директорию"
    unset start_path

    # Перезапускаем .bashrc
    print_status "Перезапускаем .bashrc..."
    source ~/.bashrc || print_error "Ошибка выполнения .bashrc"
    print_success "Перезапуск .bashrc успешно выполнен"

    # Финал
    echo -e "\n\033[1;35m======================================="
    echo " Настройка успешно завершена!          "
    echo -e "=======================================\033[0m\n"
}
