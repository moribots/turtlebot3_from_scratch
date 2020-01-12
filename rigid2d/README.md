# ME495 Sensing, Navigation, and Machine Learning
# Library: rigid2d
Author: Maurice Rahme

## Library Summary

## C.002 Questions

1. What is the difference between a `class` and a `struct` in C++?
`Classes` have default `private` `members` and `bases`, and `structs` have default `public` `members` and `bases`. Both of these can use a mixture of `private` and `public` `members` and `bases`, as well as `inheritance` and `member functions`. [Resource](https://stackoverflow.com/questions/54585/when-should-you-use-a-class-vs-a-struct-in-c/54596#54596).
2. Why is `Vector2D` a `struct` and `Transform2D` `Class`? Refer to specific C++ Core guidelines (Classes and Class Hierarchies) in your answer. (You should refer to at least 2 guidelines).
	* C.2: Use `class` if the class has an `invariant`; use `struct` if the data members can vary independently. The `x` and `y` components of a `Vector2D` can vary independently of each other. However, the `theta`, `stheta`, `ctheta`, and `x`, and `y` components of a `Transform2D` depend upon each other since a `Vector2D` can be represented in any frame, whereas a `Transform2D` represents the relationship between two frames.
	* C.8: Use `class` rather than `struct` if any member is non-public. The `theta`, `stheta`, `ctheta`, and `x`, and `y` components of a `Transform2D` are `private`, whereas the `x` and `y` components of a `Vector2D` are public. This guideline is for readability and maintenance reasons.
3. Why are some of the constructors in `Transform2D` `explicit`? Refer to specific C++ Core guidelines (Constructors, Assignments, and Destructors) in your answer.
C.46: By default, declare single-argument constructors explicit. The explicit constructors in `Transform2D` are all single-argument ones. This acts as a safeguard to ensure that the constructor is overloaded correctly.
4. We need to be able to normalize `Vector2D` objects (i.e., find the unit vector in the direction of a given `Vector2D`).Propose three different designs for implementing the normalize functionality. Discuss the pros and cons of each proposed method, in light of The C++ Core guidelines (Classes and Class Hierarchies). Which of the methods would you implement and why?

5. Implement the `normalize` functionality using the method you chose.
See `rigid2d.cpp` and `rigid2d.hpp`.
6. Why is `Transform2D::inv()` declared const while `Transform2D::operator*=()` is not?
        Refer to C++ Core Guidelines (Constants and Immutability) in your answer
The `Transform2D::inv()` member function seeks to invert a `Transform2D` without modifying the original, but instead creating a new `Transform2D` which it returns. Con.2: By default, make member functions const. A member function should be marked const unless it changes the object’s observable state. This gives a more precise statement of design intent, better readability, more errors caught by the compiler, and sometimes more optimization opportunities. By contrast, the overloaded operator `Transform2D::operator*=()` seeks to modify the passed `Transform2D` by composing it with another `Transform2D`.
