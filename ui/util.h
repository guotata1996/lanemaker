#pragma once
#include <string>
#include <filesystem>
#include <iostream>
#include <QFile>
#include <QTemporaryFile>

namespace RoadRunner
{
    std::filesystem::path DefaultSaveFolder();
    
    std::string CurrentDateTime();

    /*Unique per run*/
    std::string RunTimestamp();

    template <typename Container>
    class TQDM
    {
        using IteratorType = decltype(std::declval<Container>().cbegin());

        template <typename Iterator>
        class ProgressBarIterator {
        public:
            // Type aliases to make this wrapper compatible with STL iterator traits
            using iterator_category = typename std::iterator_traits<Iterator>::iterator_category;
            using value_type = typename std::iterator_traits<Iterator>::value_type;
            using difference_type = typename std::iterator_traits<Iterator>::difference_type;
            using pointer = typename std::iterator_traits<Iterator>::pointer;
            using reference = typename std::iterator_traits<Iterator>::reference;

            // Constructor
            ProgressBarIterator(Iterator iter, size_t size) : iter_(iter), size_(size),
                iteration(0), drawnBars(0) {}

            // Dereference operator
            reference operator*() const {
                return *iter_;
            }

            // Increment operators
            ProgressBarIterator& operator++() {  // Prefix increment
                ++iter_;
                Update();
                return *this;
            }

            ProgressBarIterator operator++(int) {  // Postfix increment
                ProgressBarIterator temp = *this;
                iter_++;
                Update();
                return temp;
            }

            // Equality operators for iterator comparison
            bool operator==(const ProgressBarIterator<Iterator>& other) const {
                return iter_ == other.iter_;
            }

            bool operator!=(const ProgressBarIterator<Iterator>& other) const {
                return iter_ != other.iter_;
            }

        private:
            void Update()
            {
                int progressToDraw = static_cast<float>(++iteration) / size_ * ProgressBarLength;
                for (int i = drawnBars; i < progressToDraw; ++i)
                {
                    std::cout << ".";
                }
                if (iteration == size_)
                {
                    std::cout << std::endl;
                }
                drawnBars = progressToDraw;
            }

            Iterator iter_;  // The actual STL iterator being wrapped
            const size_t size_;

            const int ProgressBarLength = 50;
            int iteration;
            int drawnBars;
        };

    public:
        class HelperRange : public std::vector<size_t> 
        {
        public:
            HelperRange() {};

            HelperRange(const std::vector<size_t>& v) : std::vector<size_t>(v) {};
        };
    
        TQDM(const Container& c): _size(c.size())
        {
            if (std::is_same<Container, TQDM<std::vector<size_t>>::HelperRange>::value)
            {
                helper = c;
                _beginIt = helper.cbegin();
                _endIt = helper.cend();
            }
            else
            {
                _beginIt = c.cbegin();
                _endIt = c.cend();
            }

        }

        ProgressBarIterator<IteratorType> begin() const
        {
            return ProgressBarIterator(_beginIt, _size);
        }

        ProgressBarIterator<IteratorType> end() const
        {
            return ProgressBarIterator(_endIt, _size);
        }

    private:
        IteratorType _beginIt, _endIt;
        const size_t _size;

        Container helper;
    };
    
    TQDM<std::vector<size_t>>::HelperRange range(size_t s);

    QString ExtractResourceToTempFile(const QString& resourcePath);
}
