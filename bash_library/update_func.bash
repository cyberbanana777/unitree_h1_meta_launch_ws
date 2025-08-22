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

# Функция для вывода предупреждений
print_warning() {
    echo -e "\033[1;33m[!]\033[0m $1"
}

# Функция для красочного выполнения git pull
git_pull_with_style() {
    local repo_name=$(basename "$(git rev-parse --show-toplevel 2>/dev/null)" 2>/dev/null)
    local branch=$(git symbolic-ref --short HEAD 2>/dev/null || echo "unknown")
    
    print_status "📥 Обновление репозитория: \033[1;36m$repo_name\033[0m (ветка: \033[1;35m$branch\033[0m)"
    
    # Проверяем, есть ли изменения в рабочей директории
    if [ -n "$(git status --porcelain)" ]; then
        print_warning "Есть незакоммиченные изменения:"
        git status --short
        echo
    fi
    
    # Выполняем git pull с цветным выводом
    print_status "Выполняем git pull..."
    echo -e "\033[1;30m────────────────────────────────────────\033[0m"
    
    # Запускаем git pull и захватываем вывод
    local pull_output
    if pull_output=$(git pull 2>&1); then
        echo -e "\033[1;30m────────────────────────────────────────\033[0m"
        
        # Анализируем вывод для красивого отображения
        if echo "$pull_output" | grep -q "Already up to date"; then
            print_success "Репозиторий уже актуален ✓"
        elif echo "$pull_output" | grep -q "Fast-forward"; then
            print_success "Успешно обновлено! Быстрая перемотка ✓"
            echo -e "\033[1;32mИзменения:\033[0m"
            echo "$pull_output" | grep -E "(create|delete|update)" | head -5
        else
            print_success "Обновление выполнено успешно ✓"
            echo -e "\033[1;32mРезультат:\033[0m"
            echo "$pull_output" | tail -3
        fi
    else
        echo -e "\033[1;30m────────────────────────────────────────\033[0m"
        print_error "Ошибка при выполнении git pull:"
        echo -e "\033[1;31m$pull_output\033[0m"
        return 1
    fi
    
    # Показываем актуальный статус
    print_status "Текущий статус:"
    git status --short --branch | head -3
    echo
}

# Функция, которая будет выполняться внутри каждой папки src
process_src_directory() {
    print_status "=== Обработка репозитория: \033[1;33m$(basename "$PWD")\033[0m ==="
    
    # Проверяем, что это git репозиторий
    if [ ! -d .git ]; then
        print_warning "Это не git репозиторий, пропускаем"
        return 0
    fi
    
    # Выполняем красочный git pull
    git_pull_with_style
}


update_unitree_repos() {
    cd $UNITREE_REPOS_ROOT
    print_status "Текущая рабочая директория: $PWD"
    print_status "Поиск папок unitree_h1_*..."
    echo

    # Находим все соответствующие папки и обрабатываем их
    while IFS= read -r -d $'\0' folder; do
        src_path="$folder/src"
        
        if [[ -d "$src_path" ]]; then
            print_status "Найдена папка src: $src_path"
            (
                cd "$src_path" || {
                    print_error "Ошибка перехода в $src_path" >&2
                    echo
                    exit 1
                }
                process_src_directory
            )
            echo
        else
            print_status "Пропускаю $folder - папка src не найдена" >&2
            echo
        fi
    done < <(find . -maxdepth 1 -type d -name "unitree_h1_*" -print0)

    print_success "Обработка всех папок завершена"
}