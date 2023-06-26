# MkDocs & mkdocs-material syntax test

See [mkdocs-material](https://squidfunk.github.io/mkdocs-material/reference/) for more examples

## Callouts

!!! note "Important note"
    This is a note

!!! warning
    Uh oh

## Buttons

[Open Foxglove](http://foxglove.robotics-lab.ru/){ .md-button target="_blank" }

## Code

``` py title="main.py"
import tensorflow as tf
```

## Math

$$
\operatorname{ker} f=\{g\in G:f(g)=e_{H}\}{\mbox{.}}
$$

## Images

<figure markdown>
  ![Image title](svg/linear_cart_pole.svg){ width="600" }
  <figcaption>Image caption</figcaption>
</figure>

## Diagrams

``` mermaid
sequenceDiagram
  autonumber
  Alice->>John: Hello John, how are you?
  loop Healthcheck
      John->>John: Fight against hypochondria
  end
  Note right of John: Rational thoughts!
  John-->>Alice: Great!
  John->>Bob: How about you?
  Bob-->>John: Jolly good!
```
