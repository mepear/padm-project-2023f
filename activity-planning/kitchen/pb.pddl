(define (problem pb)
  (:domain kitchen)
  (:init
    (sugar_on_stove)
    (spam_on_counter)
  )
  (:goal (and
    (sugar_on_counter)
    (spam_in_drawer)
    (not (drawer_open))
  ))
)