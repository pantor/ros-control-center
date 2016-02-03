var gulp = require('gulp'),
  babel = require('gulp-babel'),
  concat = require('gulp-concat'),
  eslint = require('gulp-eslint');

// Concat all js files into roscc.js
gulp.task('js', function () {
  return gulp.src(['app/app.js', 'app/**/*.js'])
    .pipe(babel({ presets: ['es2015'] }))
    .pipe(concat('roscc.js'))
    .pipe(gulp.dest('assets/js/'));
});

// Lint javascript based on airbnb ES5 linter and angular code style guide
gulp.task('js-lint', function() {
  return gulp.src(['app/**/*.js'])
    .pipe(eslint({
      global: ['_', 'ros', 'ROSLIB'],
      extends: ['airbnb/base', 'angular'],
      rules: {
        'no-param-reassign': 1,
        'angular/no-service-method': 0,
        'angular/controller-as-vm': 0,
      },
    }))
    .pipe(eslint.format())
    .pipe(eslint.failAfterError());
});

// Changes will be detected automatically
gulp.task('watch', function () {
  gulp.watch('app/**/*.js', ['js']);
});

gulp.task('default', ['js-lint', 'watch']);
